// SPDX-License-Identifier: AGPL-3.0-only
//! Fallible, bounded loading for the production model-import path.

use std::collections::BTreeMap;
use std::fs::File;
use std::io::Read;
use std::path::{Component, Path, PathBuf};

use super::{parse_urdf_with_config, LinkMeshAsset, Skeleton, UrdfConfig, UrdfError};

const MAX_URDF_BYTES: u64 = 16 * 1024 * 1024;
const MAX_ASSET_BYTES: u64 = 256 * 1024 * 1024;

impl Skeleton {
    /// Load a supported URDF and freeze its referenced mesh bytes.
    ///
    /// Unlike [`Self::load`], this returns an error for malformed topology,
    /// unsupported geometry, invalid motor bindings, or missing resources. Mesh
    /// references are used verbatim: GLB, OBJ, STL and DAE are supported by the
    /// Studio renderer; no converted sibling is substituted. Relative paths use
    /// the canonical URDF target's directory, including when the URDF is a
    /// symlink. `package://` references require a matching ancestor
    /// package or sibling package directory, with no guessed-package fallback.
    /// The reference's extension selects the format, even through a symlink.
    /// References to one canonical file must agree on that format.
    /// Explicit URDF colors are accepted only when they exactly duplicate used
    /// embedded DAE diffuse effects; textures and appearance overrides fail.
    ///
    /// Import is bounded to 16 MiB of XML and 256 MiB of unique asset bytes.
    /// Assets are read once and retained for logging/reconnect, so later file
    /// changes cannot silently alter the imported model. This is filesystem and
    /// model validation, not GPU decoding or live joint-state acceptance. The
    /// renderer ignores OBJ material libraries and supports only triangle
    /// geometry/diffuse materials in DAE (no textures). Referenced mesh bytes
    /// are frozen; resources internal to those formats are not resolved here.
    /// Call from the visualization worker, never a control-handler thread.
    pub fn try_load(path: &Path, cfg: &UrdfConfig) -> Result<Self, UrdfError> {
        let path = path.canonicalize().map_err(|e| resource_error(path, e))?;
        let bytes = read_resource(&path, MAX_URDF_BYTES)?;
        let xml = std::str::from_utf8(&bytes).map_err(|e| UrdfError::Xml(e.to_string()))?;
        super::validation::validate_for_loading(xml, cfg)?;
        let declarations = super::materials::declarations(xml)?;
        let mut model = parse_urdf_with_config(xml, cfg)?;
        let dir = path
            .parent()
            .ok_or_else(|| resource_error(&path, "URDF file has no parent directory"))?;
        let mut remaining = MAX_ASSET_BYTES;
        let mut mesh_formats = BTreeMap::new();
        let mut resolved = Vec::new();
        let mut required = BTreeMap::<PathBuf, Vec<&[super::materials::Declaration]>>::new();
        for (link, visual) in &model.link_visuals {
            let asset_path = resolve_resource(dir, &visual.mesh_filename)?;
            let extension = Path::new(&visual.mesh_filename)
                .extension()
                .and_then(|s| s.to_str())
                .unwrap_or("")
                .to_ascii_lowercase();
            let media_type = match extension.as_str() {
                "glb" => rerun::components::MediaType::GLB,
                "obj" => rerun::components::MediaType::OBJ,
                "stl" => rerun::components::MediaType::STL,
                "dae" => rerun::components::MediaType::DAE,
                _ => {
                    return Err(UrdfError::InvalidModel(format!(
                        "mesh reference {:?} has unsupported format; use GLB, OBJ, STL or DAE",
                        visual.mesh_filename
                    )))
                }
            };
            if let Some(previous) = mesh_formats.insert(asset_path.clone(), media_type) {
                if previous != media_type {
                    return Err(UrdfError::InvalidModel(format!(
                        "mesh reference {:?} resolves to {} with conflicting formats ({previous} and {media_type}); use one format for every reference to the same file",
                        visual.mesh_filename,
                        asset_path.display()
                    )));
                }
            }
            if let Some(materials) = declarations.get(link) {
                if extension != "dae" {
                    return Err(resource_error(
                        &asset_path,
                        "explicit URDF colors require verified embedded DAE effects",
                    ));
                }
                required
                    .entry(asset_path.clone())
                    .or_default()
                    .push(materials.as_slice());
            }
            resolved.push((link, visual, asset_path, media_type));
        }
        for (link, visual, asset_path, media_type) in resolved {
            if !model.prepared_meshes.contains_key(&asset_path) {
                let contents = read_resource(&asset_path, remaining)?;
                remaining -= contents.len() as u64;
                if let Some(materials) = required.get(&asset_path) {
                    super::materials::verify(&contents, materials)
                        .map_err(|e| resource_error(&asset_path, e))?;
                }
                let asset = rerun::Asset3D::from_file_contents(contents, Some(media_type));
                model.prepared_meshes.insert(asset_path.clone(), asset);
            }
            model.mesh_assets.push(LinkMeshAsset {
                entity: format!("{}/mesh", model.link_entity[link]),
                glb_path: asset_path,
                origin_xyz: visual.origin_xyz,
                origin_rpy: visual.origin_rpy,
                scale: visual.scale,
            });
        }
        Ok(Self {
            model: Some(model),
            ..Self::default()
        })
    }
}

fn resource_error(path: &Path, message: impl ToString) -> UrdfError {
    UrdfError::Resource {
        path: path.to_path_buf(),
        message: message.to_string(),
    }
}

fn read_resource(path: &Path, limit: u64) -> Result<Vec<u8>, UrdfError> {
    let metadata = std::fs::metadata(path).map_err(|e| resource_error(path, e))?;
    if !metadata.is_file() {
        return Err(resource_error(path, "expected a regular file"));
    }
    let mut bytes = Vec::new();
    File::open(path)
        .map_err(|e| resource_error(path, e))?
        .take(limit + 1)
        .read_to_end(&mut bytes)
        .map_err(|e| resource_error(path, e))?;
    if bytes.is_empty() || bytes.len() as u64 > limit {
        return Err(resource_error(
            path,
            format!("expected 1..={limit} bytes within the remaining import budget"),
        ));
    }
    Ok(bytes)
}

fn resolve_resource(dir: &Path, reference: &str) -> Result<PathBuf, UrdfError> {
    let path = if let Some(package_ref) = reference.strip_prefix("package://") {
        let (package, relative) = package_ref.split_once('/').ok_or_else(|| {
            UrdfError::InvalidModel(format!("invalid mesh package reference {reference:?}"))
        })?;
        let relative = Path::new(relative);
        if package.is_empty()
            || package == "."
            || package == ".."
            || relative.as_os_str().is_empty()
            || relative
                .components()
                .any(|c| !matches!(c, Component::Normal(_)))
        {
            return Err(UrdfError::InvalidModel(format!(
                "invalid mesh package reference {reference:?}"
            )));
        }
        let root = dir
            .ancestors()
            .find_map(|ancestor| {
                if ancestor.file_name().is_some_and(|name| name == package) {
                    Some(ancestor.to_path_buf())
                } else {
                    let sibling = ancestor.join(package);
                    sibling.is_dir().then_some(sibling)
                }
            })
            .ok_or_else(|| {
                UrdfError::InvalidModel(format!(
                    "cannot resolve package {package:?} for mesh {reference:?} from {}",
                    dir.display()
                ))
            })?;
        root.join(relative)
    } else {
        if reference.contains("://") {
            return Err(UrdfError::InvalidModel(format!(
                "unsupported mesh URI {reference:?}; use a file path or package:// reference"
            )));
        }
        dir.join(reference)
    };
    path.canonicalize().map_err(|e| resource_error(&path, e))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn config() -> UrdfConfig {
        UrdfConfig {
            motor_joints: Vec::new(),
            ..UrdfConfig::default()
        }
    }

    fn mesh_model(reference: &str) -> String {
        format!(
            r#"<robot name="triangle"><link name="base"><visual>
                <origin xyz="1 2 3"/><geometry><mesh filename="{reference}" scale="2 3 4"/>
                </geometry></visual></link></robot>"#
        )
    }

    // A hand-written triangle, not renderer-generated expected output.
    const TRIANGLE: &str = "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n";

    #[test]
    fn original_mesh_is_frozen_with_its_visual_transform() {
        let dir = tempfile::tempdir().unwrap();
        let mesh = dir.path().join("triangle.obj");
        let urdf = dir.path().join("robot.urdf");
        std::fs::write(&mesh, TRIANGLE).unwrap();
        std::fs::write(&urdf, mesh_model("triangle.obj")).unwrap();
        let skeleton = Skeleton::try_load(&urdf, &config()).unwrap();
        let asset = &skeleton.link_mesh_assets()[0];
        assert_eq!(asset.glb_path, mesh.canonicalize().unwrap());
        assert_eq!(asset.origin_xyz, [1.0, 2.0, 3.0]);
        assert_eq!(asset.scale, [2.0, 3.0, 4.0]);
        let prepared = &skeleton.model.as_ref().unwrap().prepared_meshes[&asset.glb_path];
        let expected = rerun::Asset3D::from_file_contents(
            TRIANGLE.as_bytes().to_vec(),
            Some(rerun::components::MediaType::OBJ),
        );
        std::fs::remove_file(&mesh).unwrap();
        assert_eq!(prepared, &expected, "import retains the original bytes");
        let (rec, storage) = rerun::RecordingStreamBuilder::new("frozen-model")
            .memory()
            .unwrap();
        skeleton.model.as_ref().unwrap().log_statics(&rec);
        rec.flush_blocking().unwrap();
        let mut blobs = Vec::new();
        for message in storage.take() {
            let rerun::log::LogMsg::ArrowMsg(_, arrow) = message else {
                continue;
            };
            let chunk = rerun::log::Chunk::from_arrow_msg(&arrow).unwrap();
            for batch in chunk.iter_component::<rerun::components::Blob>(
                rerun::Asset3D::descriptor_blob().component,
            ) {
                for blob in batch.iter() {
                    blobs.push(blob.0 .0.to_vec());
                }
            }
        }
        assert_eq!(
            blobs,
            vec![TRIANGLE.as_bytes()],
            "logged asset survives source removal"
        );
    }

    #[test]
    fn missing_original_does_not_substitute_a_glb_sibling() {
        let dir = tempfile::tempdir().unwrap();
        let urdf = dir.path().join("robot.urdf");
        std::fs::write(dir.path().join("missing.glb"), empty_glb()).unwrap();
        std::fs::write(&urdf, mesh_model("missing.obj")).unwrap();
        let error = Skeleton::try_load(&urdf, &config()).unwrap_err();
        assert!(matches!(error, UrdfError::Resource { .. }));
        assert!(error.to_string().contains("missing.obj"));
    }

    fn empty_glb() -> Vec<u8> {
        // A valid hand-written glTF 2 container with an empty scene.
        let mut json = br#"{"asset":{"version":"2.0"},"scenes":[{}],"scene":0}"#.to_vec();
        while !json.len().is_multiple_of(4) {
            json.push(b' ');
        }
        let mut glb = b"glTF".to_vec();
        glb.extend(2u32.to_le_bytes());
        glb.extend((20u32 + json.len() as u32).to_le_bytes());
        glb.extend((json.len() as u32).to_le_bytes());
        glb.extend(b"JSON");
        glb.extend(json);
        glb
    }

    #[test]
    fn native_extensions_keep_their_media_types_and_unknown_formats_fail() {
        use rerun::components::MediaType;
        let dir = tempfile::tempdir().unwrap();
        let urdf = dir.path().join("robot.urdf");
        let stl = b"solid triangle\nfacet normal 0 0 1\nouter loop\nvertex 0 0 0\nvertex 1 0 0\nvertex 0 1 0\nendloop\nendfacet\nendsolid triangle\n";
        let dae = br#"<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1"><asset><created>2026-01-01T00:00:00Z</created><modified>2026-01-01T00:00:00Z</modified></asset></COLLADA>"#;
        for (extension, contents, expected_type) in [
            ("glb", empty_glb(), MediaType::GLB),
            ("obj", TRIANGLE.as_bytes().to_vec(), MediaType::OBJ),
            ("stl", stl.to_vec(), MediaType::STL),
            ("dae", dae.to_vec(), MediaType::DAE),
        ] {
            let name = format!("asset.{extension}");
            std::fs::write(dir.path().join(&name), &contents).unwrap();
            std::fs::write(&urdf, mesh_model(&name)).unwrap();
            let loaded = Skeleton::try_load(&urdf, &config()).unwrap();
            let actual = loaded
                .model
                .as_ref()
                .unwrap()
                .prepared_meshes
                .values()
                .next()
                .unwrap();
            let expected = rerun::Asset3D::from_file_contents(contents, Some(expected_type));
            assert_eq!(actual, &expected, "media type for {extension}");
        }
        std::fs::write(dir.path().join("unknown.bin"), TRIANGLE).unwrap();
        std::fs::write(&urdf, mesh_model("unknown.bin")).unwrap();
        assert!(Skeleton::try_load(&urdf, &config())
            .unwrap_err()
            .to_string()
            .contains("unsupported format"));
    }

    #[test]
    #[cfg(unix)]
    fn symlinked_urdf_uses_target_directory_instead_of_alias_assets() {
        let dir = tempfile::tempdir().unwrap();
        let target_dir = dir.path().join("target");
        let alias_dir = dir.path().join("alias");
        std::fs::create_dir(&target_dir).unwrap();
        std::fs::create_dir(&alias_dir).unwrap();
        std::fs::write(target_dir.join("triangle.obj"), TRIANGLE).unwrap();
        std::fs::write(
            alias_dir.join("triangle.obj"),
            "v 0 0 0\nv 9 0 0\nv 0 9 0\nf 1 2 3\n",
        )
        .unwrap();
        std::fs::write(target_dir.join("robot.urdf"), mesh_model("triangle.obj")).unwrap();
        let alias = alias_dir.join("robot.urdf");
        std::os::unix::fs::symlink("../target/robot.urdf", &alias).unwrap();

        let loaded = Skeleton::try_load(&alias, &config()).unwrap();
        let expected = rerun::Asset3D::from_file_contents(
            TRIANGLE.as_bytes().to_vec(),
            Some(rerun::components::MediaType::OBJ),
        );
        assert_eq!(
            loaded
                .model
                .as_ref()
                .unwrap()
                .prepared_meshes
                .values()
                .next()
                .unwrap(),
            &expected,
            "the alias directory's same-name mesh must never replace the target mesh"
        );
        assert_eq!(
            loaded.link_mesh_assets()[0].asset_path(),
            target_dir.join("triangle.obj").canonicalize().unwrap()
        );
    }

    #[test]
    #[cfg(unix)]
    fn dangling_and_looping_urdf_symlinks_report_resource_errors() {
        let dir = tempfile::tempdir().unwrap();
        for (name, target) in [
            ("dangling.urdf", "missing.urdf"),
            ("loop.urdf", "loop.urdf"),
        ] {
            let path = dir.path().join(name);
            std::os::unix::fs::symlink(target, &path).unwrap();
            let error = Skeleton::try_load(&path, &config()).unwrap_err();
            match error {
                UrdfError::Resource {
                    path: resource,
                    message,
                } => {
                    assert_eq!(resource, path);
                    assert!(!message.is_empty());
                }
                other => panic!("expected resource error for {name}, got {other}"),
            }
        }
    }

    #[test]
    #[cfg(unix)]
    fn symlink_reference_keeps_its_format_for_an_extensionless_target() {
        assert_obj_symlink_preserves_reference_format("mesh-bytes");
    }

    #[test]
    #[cfg(unix)]
    fn symlink_reference_keeps_its_format_for_a_differently_named_target() {
        assert_obj_symlink_preserves_reference_format("mesh-bytes.stl");
    }

    #[cfg(unix)]
    fn assert_obj_symlink_preserves_reference_format(target_name: &str) {
        let dir = tempfile::tempdir().unwrap();
        let target = dir.path().join(target_name);
        std::fs::write(&target, TRIANGLE).unwrap();
        std::os::unix::fs::symlink(target_name, dir.path().join("triangle.obj")).unwrap();
        let urdf = dir.path().join("robot.urdf");
        std::fs::write(&urdf, mesh_model("triangle.obj")).unwrap();
        let loaded = Skeleton::try_load(&urdf, &config()).unwrap();
        let canonical = target.canonicalize().unwrap();
        assert_eq!(loaded.link_mesh_assets()[0].asset_path(), canonical);
        let expected = rerun::Asset3D::from_file_contents(
            TRIANGLE.as_bytes().to_vec(),
            Some(rerun::components::MediaType::OBJ),
        );
        assert_eq!(
            loaded.model.as_ref().unwrap().prepared_meshes[&canonical],
            expected
        );
    }

    #[test]
    #[cfg(unix)]
    fn conflicting_symlink_format_aliases_are_rejected_in_either_order() {
        let dir = tempfile::tempdir().unwrap();
        std::fs::write(dir.path().join("triangle.obj"), TRIANGLE).unwrap();
        std::os::unix::fs::symlink("triangle.obj", dir.path().join("triangle.stl")).unwrap();
        let urdf = dir.path().join("robot.urdf");
        for (first, second) in [
            ("triangle.obj", "triangle.stl"),
            ("triangle.stl", "triangle.obj"),
        ] {
            let xml = mesh_model(first).replace("</robot>", &format!(r#"
                <link name="tip"><visual><geometry><mesh filename="{second}"/></geometry></visual></link>
                <joint name="mount" type="fixed"><parent link="base"/><child link="tip"/></joint>
                </robot>"#));
            std::fs::write(&urdf, xml).unwrap();
            let error = Skeleton::try_load(&urdf, &config()).unwrap_err();
            assert!(matches!(error, UrdfError::InvalidModel(_)));
            assert!(error.to_string().contains("conflicting formats"), "{error}");
        }
    }

    #[test]
    #[cfg(unix)]
    fn matching_symlink_format_aliases_share_bytes_but_unknown_aliases_fail() {
        let dir = tempfile::tempdir().unwrap();
        std::fs::write(dir.path().join("triangle.obj"), TRIANGLE).unwrap();
        for alias in ["alias.OBJ", "alias.bin"] {
            std::os::unix::fs::symlink("triangle.obj", dir.path().join(alias)).unwrap();
        }
        let urdf = dir.path().join("robot.urdf");
        let xml = mesh_model("triangle.obj").replace("</robot>", r#"
            <link name="tip"><visual><geometry><mesh filename="alias.OBJ"/></geometry></visual></link>
            <joint name="mount" type="fixed"><parent link="base"/><child link="tip"/></joint>
            </robot>"#);
        std::fs::write(&urdf, xml).unwrap();
        let loaded = Skeleton::try_load(&urdf, &config()).unwrap();
        assert_eq!(loaded.link_mesh_assets().len(), 2);
        assert_eq!(loaded.model.as_ref().unwrap().prepared_meshes.len(), 1);

        std::fs::write(&urdf, mesh_model("alias.bin")).unwrap();
        let error = Skeleton::try_load(&urdf, &config()).unwrap_err();
        assert!(error.to_string().contains("unsupported format"), "{error}");
        assert!(error.to_string().contains("alias.bin"), "{error}");
    }

    #[test]
    fn repeated_references_share_one_frozen_asset() {
        let dir = tempfile::tempdir().unwrap();
        std::fs::write(dir.path().join("triangle.obj"), TRIANGLE).unwrap();
        let urdf = dir.path().join("robot.urdf");
        let xml = mesh_model("triangle.obj").replace("</robot>", r#"
            <link name="tip"><visual><geometry><mesh filename="./triangle.obj"/></geometry></visual></link>
            <joint name="mount" type="fixed"><parent link="base"/><child link="tip"/></joint>
            </robot>"#);
        std::fs::write(&urdf, xml).unwrap();
        let loaded = Skeleton::try_load(&urdf, &config()).unwrap();
        assert_eq!(loaded.link_mesh_assets().len(), 2);
        assert_eq!(loaded.model.as_ref().unwrap().prepared_meshes.len(), 1);
    }

    #[test]
    fn package_resolution_requires_the_named_package() {
        let dir = tempfile::tempdir().unwrap();
        let pkg = dir.path().join("robot_description");
        std::fs::create_dir_all(pkg.join("urdf")).unwrap();
        std::fs::create_dir_all(pkg.join("meshes")).unwrap();
        let mesh = pkg.join("meshes/triangle.obj");
        std::fs::write(&mesh, TRIANGLE).unwrap();
        let base = pkg.join("urdf");
        assert_eq!(
            resolve_resource(&base, "package://robot_description/meshes/triangle.obj").unwrap(),
            mesh.canonicalize().unwrap()
        );
        assert!(resolve_resource(&base, "package://missing/meshes/triangle.obj").is_err());
        assert!(resolve_resource(&base, "package://robot_description/../triangle.obj").is_err());
        assert!(resolve_resource(&base, "https://example.invalid/model.obj").is_err());
    }

    #[test]
    fn relative_urdf_can_resolve_package_ancestors_above_its_lexical_path() {
        // No process-global chdir: the current crate directory is the named
        // package, with an isolated child containing this test's model files.
        let cwd = std::env::current_dir().unwrap();
        let dir = tempfile::tempdir_in(&cwd).unwrap();
        let relative_dir = dir.path().strip_prefix(&cwd).unwrap();
        let package = cwd.file_name().unwrap().to_str().unwrap();
        let reference = format!(
            "package://{package}/{}/triangle.obj",
            relative_dir.display()
        );
        std::fs::write(dir.path().join("triangle.obj"), TRIANGLE).unwrap();
        std::fs::write(dir.path().join("robot.urdf"), mesh_model(&reference)).unwrap();
        let relative_urdf = relative_dir.join("robot.urdf");
        assert!(!relative_urdf.is_absolute());
        let loaded = Skeleton::try_load(&relative_urdf, &config()).unwrap();
        assert_eq!(
            loaded.link_mesh_assets()[0].asset_path(),
            dir.path().join("triangle.obj").canonicalize().unwrap()
        );
    }

    #[test]
    fn resource_bounds_reject_empty_oversize_and_directory_inputs() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("resource");
        std::fs::write(&path, b"1234").unwrap();
        assert_eq!(read_resource(&path, 4).unwrap(), b"1234");
        assert!(read_resource(&path, 3).is_err());
        std::fs::write(&path, b"").unwrap();
        assert!(read_resource(&path, 4).is_err());
        assert!(read_resource(dir.path(), 4).is_err());
    }
}
