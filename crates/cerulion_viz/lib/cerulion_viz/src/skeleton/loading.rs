// SPDX-License-Identifier: AGPL-3.0-only
//! Fallible, bounded loading for the production model-import path.

use std::collections::BTreeMap;
use std::fs::{File, OpenOptions};
use std::io::Read;
use std::path::{Component, Path, PathBuf};

use super::materials::Declaration;
use super::{parse_urdf_with_config, LinkMeshAsset, Skeleton, UrdfConfig, UrdfError};

/// The import bounds: one cap on the URDF document and one cumulative budget
/// shared by every unique mesh file of the model.
#[derive(Debug, Clone, Copy)]
struct Limits {
    /// Largest URDF document accepted, in bytes.
    urdf_bytes: u64,
    /// Total unique mesh bytes one model may freeze.
    asset_bytes: u64,
}

const PRODUCTION_LIMITS: Limits = Limits {
    urdf_bytes: 16 * 1024 * 1024,
    asset_bytes: 256 * 1024 * 1024,
};

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
        try_load_bounded(path, cfg, PRODUCTION_LIMITS)
    }
}

/// [`Skeleton::try_load`] with explicit bounds, so the budget arithmetic is
/// testable without a 256 MiB fixture.
fn try_load_bounded(path: &Path, cfg: &UrdfConfig, limits: Limits) -> Result<Skeleton, UrdfError> {
    let path = path.canonicalize().map_err(|e| resource_error(path, e))?;
    let bytes = read_bounded(&path, limits.urdf_bytes).map_err(|failure| match failure {
        ReadFailure::Oversize => resource_error(
            &path,
            format!("URDF exceeds the {}-byte import cap", limits.urdf_bytes),
        ),
        other => resource_error(&path, other),
    })?;
    let xml = std::str::from_utf8(&bytes).map_err(|e| UrdfError::Xml(e.to_string()))?;
    super::validation::validate_for_loading(xml, cfg)?;
    let declarations = super::materials::declarations(xml)?;
    let mut model = parse_urdf_with_config(xml, cfg)?;
    let dir = path
        .parent()
        .ok_or_else(|| resource_error(&path, "URDF file has no parent directory"))?;
    let mut remaining = limits.asset_bytes;
    let mut mesh_formats = BTreeMap::new();
    // Resolve every reference before reading any asset: a shared DAE must be
    // verified against the declarations of EVERY link that uses it, including
    // a link that appears after the one whose read froze the bytes.
    let mut resolved = Vec::new();
    let mut required = BTreeMap::<PathBuf, Vec<&Declaration>>::new();
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
        if let Some(declaration) = declarations.get(link) {
            if extension != "dae" {
                return Err(UrdfError::InvalidModel(format!(
                    "mesh reference {:?} declares URDF colors but is not a DAE file; explicit colors require verified embedded DAE effects",
                    visual.mesh_filename
                )));
            }
            required
                .entry(asset_path.clone())
                .or_default()
                .push(declaration);
        }
        resolved.push((link, visual, asset_path, media_type));
    }
    for (link, visual, asset_path, media_type) in resolved {
        if !model.prepared_meshes.contains_key(&asset_path) {
            let contents = read_bounded(&asset_path, remaining).map_err(|failure| match failure {
                ReadFailure::Oversize if remaining == limits.asset_bytes => resource_error(
                    &asset_path,
                    format!("mesh exceeds the {}-byte asset budget", limits.asset_bytes),
                ),
                ReadFailure::Oversize => resource_error(
                    &asset_path,
                    format!(
                        "mesh exceeds the remaining {remaining}-byte asset budget ({} bytes per model)",
                        limits.asset_bytes
                    ),
                ),
                other => resource_error(&asset_path, other),
            })?;
            remaining -= contents.len() as u64;
            if let Some(declarations) = required.get(&asset_path) {
                super::materials::verify_asset(&asset_path, &contents, declarations)?;
            }
            let asset = rerun::Asset3D::from_file_contents(contents, Some(media_type));
            model.prepared_meshes.insert(asset_path.clone(), asset);
        }
        model.mesh_assets.push(LinkMeshAsset {
            entity: format!("{}/mesh", model.link_entity[link]),
            asset_path,
            origin_xyz: visual.origin_xyz,
            origin_rpy: visual.origin_rpy,
            scale: visual.scale,
        });
    }
    Skeleton::announce(&model);
    Ok(Skeleton {
        model: Some(model),
        strict_loaded: true,
        ..Skeleton::default()
    })
}

fn resource_error(path: &Path, message: impl ToString) -> UrdfError {
    UrdfError::Resource {
        path: path.to_path_buf(),
        message: message.to_string(),
    }
}

/// Why [`read_bounded`] refused a resource. The caller names the bound that
/// was hit, because the same read serves the URDF cap and the asset budget.
#[derive(Debug)]
enum ReadFailure {
    Io(std::io::Error),
    NotRegularFile,
    Empty,
    Oversize,
}

impl std::fmt::Display for ReadFailure {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Io(e) => write!(f, "{e}"),
            Self::NotRegularFile => f.write_str("expected a regular file"),
            Self::Empty => f.write_str("empty file"),
            Self::Oversize => f.write_str("exceeds the byte limit"),
        }
    }
}

/// Open `path` as a regular file and read at most `limit` bytes from the OPEN
/// descriptor.
///
/// `open(2)` itself has side effects on a character device: a mesh reference
/// that resolves, directly or through a symlink, to a serial port toggles its
/// modem lines and can reset the microcontroller on the other end. The type
/// check must therefore run BEFORE anything opens the device, and it must be
/// pinned to the object the read uses, never to a pathname a local writer can
/// swap between two calls. [`open_regular_file`] does that per platform; the
/// `fstat` here on the descriptor the read uses is the check the read trusts.
fn read_bounded(path: &Path, limit: u64) -> Result<Vec<u8>, ReadFailure> {
    let file = open_regular_file(path)?;
    if !file.metadata().map_err(ReadFailure::Io)?.is_file() {
        return Err(ReadFailure::NotRegularFile);
    }
    let mut bytes = Vec::new();
    file.take(limit.saturating_add(1))
        .read_to_end(&mut bytes)
        .map_err(ReadFailure::Io)?;
    if bytes.is_empty() {
        return Err(ReadFailure::Empty);
    }
    if bytes.len() as u64 > limit {
        return Err(ReadFailure::Oversize);
    }
    Ok(bytes)
}

/// Take a handle to the object at `path` WITHOUT opening it, check that it is
/// a regular file, then open that same object for reading.
///
/// `O_PATH` resolves the pathname to a file description that runs no device
/// driver's open, cannot block on a FIFO and cannot claim a terminal. `fstat`
/// on it reports the object's type, and reopening through `/proc/self/fd`
/// opens that very object, not a fresh resolution of the pathname: a path
/// swapped for a device after the check is never opened.
#[cfg(target_os = "linux")]
fn open_regular_file(path: &Path) -> Result<File, ReadFailure> {
    use std::os::fd::AsRawFd;
    use std::os::unix::fs::OpenOptionsExt;
    let handle = OpenOptions::new()
        .read(true)
        .custom_flags(libc::O_PATH)
        .open(path)
        .map_err(ReadFailure::Io)?;
    if !handle.metadata().map_err(ReadFailure::Io)?.is_file() {
        return Err(ReadFailure::NotRegularFile);
    }
    OpenOptions::new()
        .read(true)
        .custom_flags(libc::O_NONBLOCK | libc::O_NOCTTY)
        .open(format!("/proc/self/fd/{}", handle.as_raw_fd()))
        .map_err(ReadFailure::Io)
}

/// Without `O_PATH` the check cannot be pinned to the object before the open,
/// so two layers stand in: a `stat` on the pathname refuses a device without
/// opening it, and the open is non-blocking and never claims a controlling
/// terminal, so a FIFO or tty swapped in between returns at once and is
/// refused by the descriptor check in [`read_bounded`]. The residual is the
/// swap window itself, which only a writer of the model directory can reach.
#[cfg(all(unix, not(target_os = "linux")))]
fn open_regular_file(path: &Path) -> Result<File, ReadFailure> {
    use std::os::unix::fs::OpenOptionsExt;
    if !std::fs::metadata(path).map_err(ReadFailure::Io)?.is_file() {
        return Err(ReadFailure::NotRegularFile);
    }
    OpenOptions::new()
        .read(true)
        .custom_flags(libc::O_NONBLOCK | libc::O_NOCTTY)
        .open(path)
        .map_err(ReadFailure::Io)
}

/// A pathname `stat` refuses a non-file before the open; the descriptor check
/// in [`read_bounded`] covers a swap after it.
#[cfg(not(unix))]
fn open_regular_file(path: &Path) -> Result<File, ReadFailure> {
    if !std::fs::metadata(path).map_err(ReadFailure::Io)?.is_file() {
        return Err(ReadFailure::NotRegularFile);
    }
    File::open(path).map_err(ReadFailure::Io)
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
        assert_eq!(asset.asset_path, mesh.canonicalize().unwrap());
        assert_eq!(asset.origin_xyz, [1.0, 2.0, 3.0]);
        assert_eq!(asset.scale, [2.0, 3.0, 4.0]);
        let prepared = &skeleton.model.as_ref().unwrap().prepared_meshes[&asset.asset_path];
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
            loaded.link_mesh_assets()[0].asset_path,
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
        assert_eq!(loaded.link_mesh_assets()[0].asset_path, canonical);
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

    /// A RELATIVE path to this test's scratch directory, built from the process
    /// working directory with `..` components only. No process-global `chdir`,
    /// nothing written into the source tree: the fixture lives under the system
    /// temp directory and is reached through its own symlink alias.
    #[cfg(unix)]
    fn relative_from_cwd(target: &Path) -> PathBuf {
        let cwd = std::env::current_dir().unwrap();
        let mut relative = PathBuf::new();
        for component in cwd.components().skip(1) {
            assert!(matches!(component, Component::Normal(_)), "cwd {cwd:?}");
            relative.push("..");
        }
        for component in target.components().skip(1) {
            relative.push(component);
        }
        relative
    }

    #[test]
    #[cfg(unix)]
    fn relative_urdf_can_resolve_package_ancestors_above_its_lexical_path() {
        // The URDF is reached through an alias symlink: `pkg` appears nowhere in
        // the lexical path handed to try_load, only in its canonical target, so a
        // loader that rooted resolution at the lexical parent would fail here.
        let dir = tempfile::tempdir().unwrap();
        let pkg = dir.path().join("pkg");
        std::fs::create_dir_all(pkg.join("urdf")).unwrap();
        std::fs::create_dir_all(pkg.join("meshes")).unwrap();
        std::fs::write(pkg.join("meshes/triangle.obj"), TRIANGLE).unwrap();
        std::fs::write(
            pkg.join("urdf/robot.urdf"),
            mesh_model("package://pkg/meshes/triangle.obj"),
        )
        .unwrap();
        std::os::unix::fs::symlink("pkg/urdf", dir.path().join("alias")).unwrap();
        let relative_urdf = relative_from_cwd(&dir.path().join("alias/robot.urdf"));
        assert!(!relative_urdf.is_absolute());
        assert!(
            relative_urdf
                .ancestors()
                .all(|a| a.file_name() != Some("pkg".as_ref())),
            "the lexical path must not name the package: {relative_urdf:?}"
        );
        let loaded = Skeleton::try_load(&relative_urdf, &config()).unwrap();
        assert_eq!(
            loaded.link_mesh_assets()[0].asset_path,
            pkg.join("meshes/triangle.obj").canonicalize().unwrap()
        );
    }

    #[test]
    fn read_bounded_names_empty_oversize_and_directory_inputs() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("resource");
        std::fs::write(&path, b"1234").unwrap();
        assert_eq!(read_bounded(&path, 4).unwrap(), b"1234");
        assert!(matches!(read_bounded(&path, 3), Err(ReadFailure::Oversize)));
        std::fs::write(&path, b"").unwrap();
        assert!(matches!(read_bounded(&path, 4), Err(ReadFailure::Empty)));
        assert!(matches!(
            read_bounded(dir.path(), 4),
            Err(ReadFailure::NotRegularFile)
        ));
        assert!(matches!(
            read_bounded(&dir.path().join("absent"), 4),
            Err(ReadFailure::Io(_))
        ));
    }

    /// A mesh reference that resolves through a symlink to a character device
    /// is refused before anything opens the device.
    #[test]
    #[cfg(unix)]
    fn a_symlink_to_a_character_device_is_refused_without_opening_it() {
        let dir = tempfile::tempdir().unwrap();
        let link = dir.path().join("triangle.obj");
        std::os::unix::fs::symlink("/dev/null", &link).unwrap();
        assert!(matches!(
            read_bounded(&link, 4),
            Err(ReadFailure::NotRegularFile)
        ));
        let urdf = dir.path().join("robot.urdf");
        std::fs::write(&urdf, mesh_model("triangle.obj")).unwrap();
        let error = Skeleton::try_load(&urdf, &config()).unwrap_err();
        assert!(matches!(error, UrdfError::Resource { .. }), "{error}");
        assert!(
            error.to_string().contains("expected a regular file"),
            "{error}"
        );
    }

    #[test]
    #[cfg(unix)]
    fn a_fifo_in_place_of_a_mesh_is_refused_without_blocking() {
        use std::os::unix::ffi::OsStrExt;
        let dir = tempfile::tempdir().unwrap();
        let fifo = dir.path().join("triangle.obj");
        let c_path = std::ffi::CString::new(fifo.as_os_str().as_bytes()).unwrap();
        // SAFETY: `c_path` is a valid NUL-terminated C string that outlives the call.
        assert_eq!(unsafe { libc::mkfifo(c_path.as_ptr(), 0o600) }, 0);
        let urdf = dir.path().join("robot.urdf");
        std::fs::write(&urdf, mesh_model("triangle.obj")).unwrap();
        // No writer ever attaches: a blocking open would park this thread for
        // good, so the load runs on a helper and the assertion waits a bounded
        // time for its verdict instead of hanging the whole test binary.
        let (tx, rx) = std::sync::mpsc::channel();
        let loader = std::thread::spawn(move || {
            tx.send(Skeleton::try_load(&urdf, &config()).map(|_| ()))
                .unwrap();
        });
        let outcome = rx
            .recv_timeout(std::time::Duration::from_secs(10))
            .expect("the strict loader must refuse a FIFO instead of blocking on it");
        loader.join().unwrap();
        let error = outcome.unwrap_err();
        assert!(matches!(error, UrdfError::Resource { .. }), "{error}");
        assert!(
            error.to_string().contains("expected a regular file"),
            "{error}"
        );
    }

    const TINY: Limits = Limits {
        urdf_bytes: 4096,
        asset_bytes: 15,
    };

    /// Two 10-byte meshes against a 15-byte budget: the first fits, the second
    /// is refused by the CUMULATIVE budget even though it is under the cap alone.
    #[test]
    fn asset_budget_is_cumulative_across_distinct_meshes() {
        let dir = tempfile::tempdir().unwrap();
        let ten_bytes = "v 0 0 0\nf ";
        assert_eq!(ten_bytes.len(), 10);
        std::fs::write(dir.path().join("a.obj"), ten_bytes).unwrap();
        std::fs::write(dir.path().join("b.obj"), ten_bytes).unwrap();
        let urdf = dir.path().join("robot.urdf");
        let two = mesh_model("a.obj").replace(
            "</robot>",
            r#"
            <link name="tip"><visual><geometry><mesh filename="b.obj"/></geometry></visual></link>
            <joint name="mount" type="fixed"><parent link="base"/><child link="tip"/></joint>
            </robot>"#,
        );
        std::fs::write(&urdf, two).unwrap();
        let error = try_load_bounded(&urdf, &config(), TINY).unwrap_err();
        match &error {
            UrdfError::Resource { path, message } => {
                assert_eq!(path, &dir.path().join("b.obj").canonicalize().unwrap());
                assert_eq!(
                    message,
                    "mesh exceeds the remaining 5-byte asset budget (15 bytes per model)"
                );
            }
            other => panic!("expected a budget error, got {other}"),
        }

        // The same two references to ONE file share bytes and stay within budget.
        let shared = mesh_model("a.obj").replace(
            "</robot>",
            r#"
            <link name="tip"><visual><geometry><mesh filename="./a.obj"/></geometry></visual></link>
            <joint name="mount" type="fixed"><parent link="base"/><child link="tip"/></joint>
            </robot>"#,
        );
        std::fs::write(&urdf, shared).unwrap();
        let loaded = try_load_bounded(&urdf, &config(), TINY).unwrap();
        assert_eq!(loaded.link_mesh_assets().len(), 2);
        assert_eq!(loaded.model.as_ref().unwrap().prepared_meshes.len(), 1);

        // A single mesh over the whole budget names the cap, not a remainder.
        std::fs::write(dir.path().join("big.obj"), "v 0 0 0\nv 1 0 0\n").unwrap();
        std::fs::write(&urdf, mesh_model("big.obj")).unwrap();
        let error = try_load_bounded(&urdf, &config(), TINY).unwrap_err();
        assert!(
            error
                .to_string()
                .contains("mesh exceeds the 15-byte asset budget"),
            "{error}"
        );
    }

    #[test]
    fn oversize_urdf_is_refused_before_any_asset_is_read() {
        let dir = tempfile::tempdir().unwrap();
        let urdf = dir.path().join("robot.urdf");
        let xml = mesh_model("absent.obj");
        std::fs::write(&urdf, &xml).unwrap();
        let limits = Limits {
            urdf_bytes: xml.len() as u64 - 1,
            asset_bytes: 15,
        };
        let error = try_load_bounded(&urdf, &config(), limits).unwrap_err();
        match &error {
            UrdfError::Resource { path, message } => {
                assert_eq!(path, &urdf.canonicalize().unwrap());
                assert_eq!(
                    message,
                    &format!("URDF exceeds the {}-byte import cap", xml.len() - 1)
                );
            }
            other => panic!("expected the XML cap, got {other}"),
        }
        // One byte more and the document is read; the missing mesh is then the
        // first error, which proves the cap fired before any asset lookup.
        let limits = Limits {
            urdf_bytes: xml.len() as u64,
            asset_bytes: 15,
        };
        let error = try_load_bounded(&urdf, &config(), limits).unwrap_err();
        assert!(error.to_string().contains("absent.obj"), "{error}");
    }

    #[test]
    fn strict_loading_twice_is_bit_identical() {
        let dir = tempfile::tempdir().unwrap();
        std::fs::write(dir.path().join("triangle.obj"), TRIANGLE).unwrap();
        std::fs::write(
            dir.path().join("second.obj"),
            "v 0 0 0\nv 2 0 0\nv 0 2 0\nf 1 2 3\n",
        )
        .unwrap();
        let urdf = dir.path().join("robot.urdf");
        let xml = mesh_model("triangle.obj").replace("</robot>", r#"
            <link name="tip"><visual><geometry><mesh filename="second.obj"/></geometry></visual></link>
            <joint name="mount" type="fixed"><parent link="base"/><child link="tip"/></joint>
            </robot>"#);
        std::fs::write(&urdf, xml).unwrap();
        let first = Skeleton::try_load(&urdf, &config()).unwrap();
        let second = Skeleton::try_load(&urdf, &config()).unwrap();
        assert_eq!(first.link_mesh_assets(), second.link_mesh_assets());
        assert_eq!(
            first.model.as_ref().unwrap().prepared_meshes,
            second.model.as_ref().unwrap().prepared_meshes
        );
        assert_eq!(first.link_mesh_assets().len(), 2);
    }

    /// A strict load emits the same lifecycle line the legacy constructors do.
    #[tracing_test::traced_test]
    #[test]
    fn strict_loading_announces_the_active_model() {
        let dir = tempfile::tempdir().unwrap();
        std::fs::write(dir.path().join("triangle.obj"), TRIANGLE).unwrap();
        let urdf = dir.path().join("robot.urdf");
        std::fs::write(&urdf, mesh_model("triangle.obj")).unwrap();
        Skeleton::try_load(&urdf, &config()).unwrap();
        assert!(logs_contain("stick-figure archetype ACTIVE"));
        assert!(logs_contain("no lidar-mount link found"));
    }
}
