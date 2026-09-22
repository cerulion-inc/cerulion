// SPDX-License-Identifier: AGPL-3.0-only
//! Prove that URDF colors duplicate used DAE diffuse effects. This never edits
//! asset bytes or applies appearance overrides, and does not validate GPU decoding.

use std::collections::{BTreeMap, BTreeSet};

use roxmltree::{Document, Node};

use super::UrdfError;

type Color = [f64; 4];

pub(super) struct Declaration {
    name: String,
    color: Color,
}

fn error(message: impl std::fmt::Display) -> UrdfError {
    UrdfError::InvalidModel(format!("embedded DAE material verification: {message}"))
}

fn attribute<'a>(node: Node<'a, '_>, name: &str) -> Result<&'a str, UrdfError> {
    node.attribute(name)
        .filter(|s| !s.trim().is_empty())
        .ok_or_else(|| error(format!("<{}> requires {name}", node.tag_name().name())))
}

fn child<'a, 'i>(node: Node<'a, 'i>, tag: &str) -> Result<Node<'a, 'i>, UrdfError> {
    let mut nodes = node.children().filter(|n| n.has_tag_name(tag));
    let result = nodes
        .next()
        .ok_or_else(|| error(format!("missing <{tag}>")))?;
    if nodes.next().is_some() {
        return Err(error(format!("ambiguous duplicate <{tag}>")));
    }
    Ok(result)
}

fn color(value: &str) -> Result<Color, UrdfError> {
    let values: Vec<f64> = value
        .split_whitespace()
        .map(str::parse)
        .collect::<Result<_, _>>()
        .map_err(|_| error("RGBA must contain four finite numbers in [0, 1]"))?;
    let values: Color = values
        .try_into()
        .map_err(|_| error("RGBA requires exactly four components"))?;
    if values
        .iter()
        .any(|v| !v.is_finite() || !(0.0..=1.0).contains(v))
    {
        return Err(error("RGBA must contain four finite numbers in [0, 1]"));
    }
    Ok(values)
}

/// Read declarations only after topology/visual preflight has established unique links.
pub(super) fn declarations(xml: &str) -> Result<BTreeMap<String, Vec<Declaration>>, UrdfError> {
    let doc = Document::parse(xml).map_err(error)?;
    let mut result = BTreeMap::new();
    let mut count = 0;
    for link in doc
        .root_element()
        .children()
        .filter(|n| n.has_tag_name("link"))
    {
        let mut declarations = Vec::new();
        let mut names = BTreeSet::new();
        for material in link
            .children()
            .filter(|n| n.has_tag_name("visual"))
            .flat_map(|v| v.children().filter(|n| n.has_tag_name("material")))
        {
            count += 1;
            if count > 4096 {
                return Err(error(
                    "at most 4096 URDF material declarations are supported",
                ));
            }
            let name = attribute(material, "name")?;
            if !names.insert(name) {
                return Err(error(format!("ambiguous repeated URDF material {name:?}")));
            }
            if material.attributes().any(|a| a.name() != "name")
                || material
                    .children()
                    .filter(|n| n.is_element())
                    .any(|n| !n.has_tag_name("color"))
            {
                return Err(error(
                    "only an inline named RGBA color can be verified; textures and material references are unsupported",
                ));
            }
            let rgba = child(material, "color")?;
            if rgba.attributes().any(|a| a.name() != "rgba")
                || rgba.children().any(|n| n.is_element())
            {
                return Err(error("unsupported URDF color contents"));
            }
            declarations.push(Declaration {
                name: name.into(),
                color: color(attribute(rgba, "rgba")?)?,
            });
        }
        if !declarations.is_empty() {
            result.insert(attribute(link, "name")?.into(), declarations);
        }
    }
    Ok(result)
}

fn fragment(node: Node<'_, '_>, attr: &str) -> Result<String, UrdfError> {
    attribute(node, attr)?
        .strip_prefix('#')
        .filter(|s| !s.is_empty())
        .map(str::to_owned)
        .ok_or_else(|| error("DAE references must name a local #id"))
}

fn lookup<'a, 'i>(
    ids: &BTreeMap<&str, Node<'a, 'i>>,
    id: &str,
    tag: &str,
) -> Result<Node<'a, 'i>, UrdfError> {
    let library = match tag {
        "geometry" => "library_geometries",
        "material" => "library_materials",
        "effect" => "library_effects",
        _ => return Err(error("unsupported DAE reference kind")),
    };
    ids.get(id)
        .copied()
        .filter(|n| {
            n.has_tag_name(tag)
                && n.parent().is_some_and(|p| {
                    p.has_tag_name(library) && p.parent() == Some(n.document().root_element())
                })
        })
        .ok_or_else(|| error(format!("unresolved {tag} {id:?}")))
}

fn effect_color(effect: Node<'_, '_>) -> Result<Color, UrdfError> {
    let technique = child(child(effect, "profile_COMMON")?, "technique")?;
    let mut shaders = technique.children().filter(|n| n.is_element());
    let shader = shaders
        .next()
        .ok_or_else(|| error("missing diffuse shader"))?;
    if shaders.next().is_some()
        || !matches!(shader.tag_name().name(), "lambert" | "phong" | "blinn")
    {
        return Err(error(
            "one Lambert, Phong, or Blinn diffuse shader is required",
        ));
    }
    let diffuse = child(shader, "diffuse")?;
    let rgba = child(diffuse, "color")?;
    if diffuse.children().filter(|n| n.is_element()).count() != 1
        || rgba.children().any(|n| n.is_element())
    {
        return Err(error("diffuse must contain only an RGBA color"));
    }
    color(rgba.text().unwrap_or(""))
}

// Match the decoder's direct node/child-node traversal; instances inside extras
// or other ignored XML must not make an otherwise-unused effect admissible.
fn instances<'a, 'i>(scene: Node<'a, 'i>) -> Result<Vec<Node<'a, 'i>>, UrdfError> {
    let mut pending: Vec<_> = scene
        .children()
        .filter(|n| n.has_tag_name("node"))
        .collect();
    let mut instances = Vec::new();
    let mut count = 0;
    while let Some(node) = pending.pop() {
        count += 1;
        if count > 4096 {
            return Err(error(
                "at most 4096 DAE scene nodes are supported for material verification",
            ));
        }
        instances.extend(
            node.children()
                .filter(|n| n.has_tag_name("instance_geometry")),
        );
        pending.extend(node.children().filter(|n| n.has_tag_name("node")));
    }
    Ok(instances)
}

/// The native decoder currently renders every visual-scene definition. Limit
/// proof to one explicitly selected scene until the decoder honors scene selection.
fn selected_scene<'a, 'i>(root: Node<'a, 'i>) -> Result<Node<'a, 'i>, UrdfError> {
    let mut scenes = root
        .children()
        .filter(|n| n.has_tag_name("library_visual_scenes"))
        .flat_map(|n| n.children().filter(|n| n.has_tag_name("visual_scene")));
    let scene = scenes
        .next()
        .ok_or_else(|| error("exactly one DAE visual-scene definition is required"))?;
    if scenes.next().is_some() {
        return Err(error("exactly one DAE visual-scene definition is supported until the native decoder honors scene selection"));
    }
    let selected = fragment(
        child(child(root, "scene")?, "instance_visual_scene")?,
        "url",
    )?;
    if selected != attribute(scene, "id")? {
        return Err(error(format!(
            "unresolved visual-scene selection {selected:?}; it must select the sole definition"
        )));
    }
    Ok(scene)
}

/// Check the same bytes that will be retained in Asset3D. Restrict acceptance to
/// the native decoder's proven subset: metres and identity symbol-to-ID bindings.
pub(super) fn verify(bytes: &[u8], required: &[&[Declaration]]) -> Result<(), UrdfError> {
    let xml = std::str::from_utf8(bytes).map_err(error)?;
    // roxmltree reserves arrays from raw delimiter counts before checking nodes_limit.
    // Bound those estimates, including delimiters inside text and comments.
    let mut openings = 0;
    let mut equals = 0;
    for byte in bytes {
        match byte {
            b'<' => openings += 1,
            b'=' => equals += 1,
            _ => {}
        }
        if openings > 131_072 || equals > 262_144 {
            return Err(error(
                "DAE parser budget exceeded: at most 131072 '<' and 262144 '=' bytes",
            ));
        }
    }
    // Bound every subsequent traversal and ID index by actual document nodes.
    let doc = Document::parse_with_options(
        xml,
        roxmltree::ParsingOptions {
            nodes_limit: 65_536,
            ..Default::default()
        },
    )
    .map_err(|cause| match cause {
        roxmltree::Error::NodesLimitReached => {
            error("at most 65536 XML nodes are supported per DAE document")
        }
        other => error(other),
    })?;
    let root = doc.root_element();
    if !root.has_tag_name("COLLADA")
        || root.tag_name().namespace() != Some("http://www.collada.org/2005/11/COLLADASchema")
    {
        return Err(error("material declarations require a COLLADA document"));
    }
    // dae-parser, used by the native renderer, accepts only this version.
    if root.attribute("version") != Some("1.4.1") {
        return Err(error(
            "material verification requires COLLADA version 1.4.1",
        ));
    }
    let mut ids = BTreeMap::new();
    for node in doc.descendants().filter(|n| n.is_element()) {
        if let Some(id) = node.attribute("id") {
            if ids.insert(id, node).is_some() {
                return Err(error(format!("ambiguous duplicate DAE id {id:?}")));
            }
        }
        if node.has_tag_name("image") || node.has_tag_name("texture") {
            return Err(error("DAE textures are unsupported"));
        }
        if node.has_tag_name("unit") && attribute(node, "meter")?.parse::<f64>().ok() != Some(1.0) {
            return Err(error(
                "the native DAE decoder requires unit meter=1 for material-verified imports",
            ));
        }
    }
    let mut used = BTreeMap::new();
    for instance in instances(selected_scene(root)?)? {
        let geometry_id = fragment(instance, "url")?;
        let geometry = lookup(&ids, &geometry_id, "geometry")?;
        let mesh = child(geometry, "mesh")?;
        let bindings = child(child(instance, "bind_material")?, "technique_common")?;
        let mut symbols = BTreeMap::new();
        for binding in bindings
            .children()
            .filter(|n| n.has_tag_name("instance_material"))
        {
            let symbol = attribute(binding, "symbol")?;
            let target = fragment(binding, "target")?;
            if target != symbol {
                return Err(error(
                    "DAE material symbol rebinding is unsupported by the native decoder",
                ));
            }
            if symbols.insert(symbol, target).is_some() {
                return Err(error(format!("ambiguous material symbol {symbol:?}")));
            }
        }
        for triangles in mesh.children().filter(|n| n.has_tag_name("triangles")) {
            if attribute(triangles, "count")?
                .parse::<u64>()
                .ok()
                .filter(|n| *n > 0)
                .is_none()
                || child(triangles, "p")?
                    .text()
                    .is_none_or(|s| s.trim().is_empty())
            {
                return Err(error("material proof requires nonempty triangle groups"));
            }
            let symbol = attribute(triangles, "material")?;
            let material_id = symbols
                .get(symbol)
                .ok_or_else(|| error(format!("unresolved triangle material symbol {symbol:?}")))?;
            let material = lookup(&ids, material_id, "material")?;
            let effect_id = fragment(child(material, "instance_effect")?, "url")?;
            let effect = lookup(&ids, &effect_id, "effect")?;
            used.insert(effect_id, effect_color(effect)?);
        }
    }
    for visual in required {
        for declaration in *visual {
            let embedded = used.get(&declaration.name).ok_or_else(|| {
                error(format!(
                    "URDF material {:?} must name a used embedded diffuse effect",
                    declaration.name
                ))
            })?;
            if embedded != &declaration.color {
                return Err(error(format!(
                "URDF RGBA for {:?} differs from its embedded diffuse color; appearance overrides are unsupported",
                declaration.name
            )));
            }
        }
        // Names are unique within each visual from declarations(). A subset
        // could express a whole-visual override, rather than redundant metadata.
        if visual.len() != used.len() {
            return Err(error("each material-bearing visual must declare the complete set of used embedded diffuse effects"));
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::skeleton::{Skeleton, UrdfConfig};

    // One handwritten red triangle with an explicit material/effect binding.
    const DAE: &str = r##"<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
      <asset><created>2026-01-01T00:00:00Z</created><modified>2026-01-01T00:00:00Z</modified><unit meter="1"/><up_axis>Z_UP</up_axis></asset>
      <library_effects><effect id="red-effect"><profile_COMMON><technique sid="common"><lambert><diffuse><color>1 0 0 1</color></diffuse></lambert></technique></profile_COMMON></effect></library_effects>
      <library_materials><material id="red-material"><instance_effect url="#red-effect"/></material></library_materials>
      <library_geometries><geometry id="triangle"><mesh>
      <source id="positions"><float_array id="positions-array" count="9">0 0 0 1 0 0 0 1 0</float_array><technique_common><accessor source="#positions-array" count="3" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>
      <vertices id="vertices"><input semantic="POSITION" source="#positions"/></vertices>
      <triangles count="1" material="red-material"><input semantic="VERTEX" source="#vertices" offset="0"/><p>0 1 2</p></triangles>
      </mesh></geometry></library_geometries>
      <library_visual_scenes><visual_scene id="scene"><node><matrix>1 0 0 2 0 1 0 3 0 0 1 4 0 0 0 1</matrix><instance_geometry url="#triangle"><bind_material><technique_common><instance_material symbol="red-material" target="#red-material"/></technique_common></bind_material></instance_geometry></node></visual_scene></library_visual_scenes>
      <scene><instance_visual_scene url="#scene"/></scene></COLLADA>"##;

    fn urdf(material: &str) -> String {
        format!(
            r#"<robot name="triangle"><link name="base"><visual><geometry><mesh filename="triangle.dae"/></geometry>{material}</visual></link></robot>"#
        )
    }

    const RED: &str = r#"<material name="red-effect"><color rgba="1 0 0 1"/></material>"#;
    const BLUE: &str = r#"<material name="blue-effect"><color rgba="0 0 1 1"/></material>"#;

    // Two separate handwritten triangles, each using its own diffuse effect.
    fn two_color_dae() -> String {
        DAE.replace("count=\"9\"", "count=\"18\"")
            .replace("count=\"3\" stride", "count=\"6\" stride")
            .replace(">0 0 0 1 0 0 0 1 0</float_array>", ">0 0 0 1 0 0 0 1 0 2 0 0 3 0 0 2 1 0</float_array>")
            .replace("</library_effects>", r#"<effect id="blue-effect"><profile_COMMON><technique sid="common"><lambert><diffuse><color>0 0 1 1</color></diffuse></lambert></technique></profile_COMMON></effect></library_effects>"#)
            .replace("</library_materials>", r##"<material id="blue-material"><instance_effect url="#blue-effect"/></material></library_materials>"##)
            .replace("</mesh>", r##"<triangles count="1" material="blue-material"><input semantic="VERTEX" source="#vertices" offset="0"/><p>3 4 5</p></triangles></mesh>"##)
            .replace("</technique_common></bind_material>", r##"<instance_material symbol="blue-material" target="#blue-material"/></technique_common></bind_material>"##)
    }

    #[test]
    fn every_declaring_visual_must_cover_the_complete_asset_effect_set() {
        let dae = two_color_dae();
        let complete = declarations(&urdf(&format!("{BLUE}{RED}"))).unwrap();
        assert_eq!(
            verify(dae.as_bytes(), &[complete["base"].as_slice()]),
            Ok(())
        );
        assert!(verify_red(&dae)
            .unwrap_err()
            .to_string()
            .contains("complete set"));

        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("model.urdf");
        std::fs::write(dir.path().join("triangle.dae"), &dae).unwrap();
        let cfg = UrdfConfig {
            motor_joints: Vec::new(),
            ..UrdfConfig::default()
        };
        let first = urdf(RED).replace("</robot>", "");
        let second = format!(
            r#"<link name="tip"><visual><geometry><mesh filename="./triangle.dae"/></geometry>{BLUE}</visual></link><joint name="fixed" type="fixed"><parent link="base"/><child link="tip"/></joint></robot>"#
        );
        std::fs::write(&path, format!("{first}{second}")).unwrap();
        assert!(Skeleton::try_load(&path, &cfg)
            .unwrap_err()
            .to_string()
            .contains("complete set"));

        let first = urdf(&format!("{RED}{BLUE}")).replace("</robot>", "");
        let second = second.replace(BLUE, &format!("{BLUE}{RED}"));
        std::fs::write(&path, format!("{first}{second}")).unwrap();
        let loaded = Skeleton::try_load(&path, &cfg).unwrap();
        assert_eq!(loaded.model.as_ref().unwrap().prepared_meshes.len(), 1);
    }

    fn verify_red(dae: &str) -> Result<(), UrdfError> {
        let declarations = declarations(&urdf(RED))?;
        verify(dae.as_bytes(), &[declarations["base"].as_slice()])
    }

    #[test]
    fn bounds_dae_parser_reservations_even_in_comments() {
        for (delimiter, limit) in [(b'<', 131_072), (b'=', 262_144)] {
            let template = DAE.replace("</COLLADA>", "<!--padding--></COLLADA>");
            let existing = template.bytes().filter(|byte| *byte == delimiter).count();
            let padding = char::from(delimiter).to_string().repeat(limit - existing);
            let at_limit = template.replace("padding", &padding);
            verify_red(&at_limit).unwrap();
            let over_limit =
                template.replace("padding", &format!("{padding}{}", char::from(delimiter)));
            assert!(verify_red(&over_limit)
                .unwrap_err()
                .to_string()
                .contains("DAE parser budget exceeded"));
        }
    }

    #[test]
    fn bounds_all_dae_document_nodes_before_material_indexing() {
        // Even unused metadata consumes parser and ID-index storage.
        let baseline = Document::parse(DAE).unwrap().descendants().count();
        let padding = (0..65_536 - baseline)
            .map(|index| format!("<extra id=\"unused-{index}\"/>"))
            .collect::<String>();
        let at_limit = DAE.replace("</COLLADA>", &format!("{padding}</COLLADA>"));
        verify_red(&at_limit).unwrap();
        let over_limit = at_limit.replace("</COLLADA>", "<extra/></COLLADA>");
        assert!(verify_red(&over_limit)
            .unwrap_err()
            .to_string()
            .contains("65536 XML nodes"));
    }

    #[test]
    fn rejects_inactive_scene_definitions_even_with_complete_materials() {
        let dae = two_color_dae();
        let declarations = declarations(&urdf(&format!("{RED}{BLUE}"))).unwrap();
        assert_eq!(
            verify(dae.as_bytes(), &[declarations["base"].as_slice()]),
            Ok(())
        );
        // Both definitions instance the same two-colored geometry. Complete
        // material membership cannot make rendering the inactive copy correct.
        let scene = dae
            .split_once("<visual_scene")
            .unwrap()
            .1
            .split_once("</visual_scene>")
            .unwrap()
            .0;
        let alternate = format!(
            "<visual_scene{}</visual_scene>",
            scene.replacen("id=\"scene\"", "id=\"inactive\"", 1)
        );
        let multiple = dae.replace(
            "</library_visual_scenes>",
            &format!("{alternate}</library_visual_scenes>"),
        );
        let error = verify(multiple.as_bytes(), &[declarations["base"].as_slice()]).unwrap_err();
        assert!(
            error.to_string().contains("exactly one DAE visual-scene"),
            "{error}"
        );
        // Count definitions across separate libraries, not just the first one.
        let separate = dae.replace("</library_visual_scenes>", &format!("</library_visual_scenes><library_visual_scenes>{alternate}</library_visual_scenes>"));
        assert!(
            verify(separate.as_bytes(), &[declarations["base"].as_slice()])
                .unwrap_err()
                .to_string()
                .contains("exactly one DAE visual-scene")
        );
    }

    #[test]
    fn requires_one_resolved_top_level_visual_scene_selection() {
        const SELECTION: &str = r##"<scene><instance_visual_scene url="#scene"/></scene>"##;
        for (source, replacement, reason) in [
            (
                "url=\"#scene\"",
                "url=\"#missing\"",
                "unresolved visual-scene selection",
            ),
            (
                "url=\"#scene\"",
                "url=\"#triangle\"",
                "unresolved visual-scene selection",
            ),
            ("url=\"#scene\"", "url=\"external.dae#scene\"", "local #id"),
            (SELECTION, "", "missing <scene>"),
            (SELECTION, "<scene/>", "missing <instance_visual_scene>"),
            (
                "<instance_visual_scene url=\"#scene\"/>",
                "<instance_visual_scene url=\"#scene\"/><instance_visual_scene url=\"#scene\"/>",
                "duplicate <instance_visual_scene>",
            ),
            ("id=\"scene\"", "", "requires id"),
        ] {
            let error = verify_red(&DAE.replace(source, replacement)).unwrap_err();
            assert!(error.to_string().contains(reason), "{error}");
        }
        let duplicate = DAE.replace(SELECTION, &format!("{SELECTION}{SELECTION}"));
        assert!(verify_red(&duplicate)
            .unwrap_err()
            .to_string()
            .contains("duplicate <scene>"));
        let missing = DAE
            .replace("<visual_scene id=\"scene\">", "<extra>")
            .replace("</visual_scene>", "</extra>");
        assert!(verify_red(&missing)
            .unwrap_err()
            .to_string()
            .contains("exactly one DAE visual-scene"));
    }

    #[test]
    fn accepts_only_exact_used_embedded_colors() {
        assert_eq!(verify_red(DAE), Ok(()));
        for (source, replacement, reason) in [
            (
                "<color>1 0 0 1</color>",
                "<color>0 1 0 1</color>",
                "differs",
            ),
            ("url=\"#red-effect\"", "url=\"#absent\"", "unresolved"),
            (
                "<triangles count=\"1\"",
                "<triangles count=\"0\"",
                "nonempty",
            ),
            (
                "<instance_geometry url=\"#triangle\">",
                "<instance_geometry url=\"#absent\">",
                "unresolved",
            ),
        ] {
            let error = verify_red(&DAE.replace(source, replacement)).unwrap_err();
            assert!(error.to_string().contains(reason), "{error}");
        }
        let unused = DAE;
        // Removing the instance, rather than its definition, makes the effect unused.
        let begin = unused.find("<instance_geometry").unwrap();
        let end = unused.find("</instance_geometry>").unwrap() + "</instance_geometry>".len();
        let unused = format!("{}{}", &unused[..begin], &unused[end..]);
        assert!(verify_red(&unused)
            .unwrap_err()
            .to_string()
            .contains("used embedded"));
        let ignored = DAE
            .replace("<node>", "<extra>")
            .replace("</node>", "</extra>");
        assert!(verify_red(&ignored)
            .unwrap_err()
            .to_string()
            .contains("used embedded"));
    }

    #[test]
    fn rejects_ambiguous_and_unsupported_dae_metadata() {
        for (source, replacement, reason) in [
            ("version=\"1.4.1\"", "version=\"1.5.0\"", "version 1.4.1"),
            (
                "<library_materials>",
                "<library_materials><material id=\"red-material\"/>",
                "duplicate DAE id",
            ),
            ("meter=\"1\"", "meter=\"0.01\"", "meter=1"),
            ("symbol=\"red-material\"", "symbol=\"surface\"", "rebinding"),
            (
                "<instance_material symbol=\"red-material\" target=\"#red-material\"/>",
                "<instance_material symbol=\"red-material\" target=\"#red-material\"/><instance_material symbol=\"red-material\" target=\"#red-material\"/>",
                "ambiguous material symbol",
            ),
            (
                "<library_effects>",
                "<library_images><image id=\"texture\"/></library_images><library_effects>",
                "textures",
            ),
            (
                "<color>1 0 0 1</color>",
                "<texture texture=\"sampler\"/>",
                "textures",
            ),
        ] {
            let error = verify_red(&DAE.replace(source, replacement)).unwrap_err();
            assert!(error.to_string().contains(reason), "{error}");
        }
    }

    #[test]
    fn rejects_missing_repeated_or_malformed_urdf_declarations() {
        for material in [
            "<material/>",
            "<material name=\"red-effect\"/>",
            "<material name=\"red-effect\"><texture filename=\"red.png\"/></material>",
            "<material name=\"red-effect\"><color rgba=\"NaN 0 0 1\"/></material>",
            "<material name=\"red-effect\"><color rgba=\"1 0 0\"/></material>",
            "<material name=\"red-effect\"><color rgba=\"2 0 0 1\"/></material>",
        ] {
            assert!(declarations(&urdf(material)).is_err(), "{material}");
        }
        assert!(declarations(&urdf(&format!("{RED}{RED}"))).is_err());
        let missing = declarations(&urdf(&RED.replace("red-effect", "missing-effect"))).unwrap();
        assert!(verify(DAE.as_bytes(), &[missing["base"].as_slice()])
            .unwrap_err()
            .to_string()
            .contains("used embedded"));
    }

    #[test]
    fn bounds_material_declarations_and_scene_traversal() {
        let materials = (0..4097)
            .map(|i| format!(r#"<material name="color{i}"><color rgba="1 0 0 1"/></material>"#))
            .collect::<String>();
        assert!(declarations(&urdf(&materials))
            .err()
            .unwrap()
            .to_string()
            .contains("4096 URDF"));
        let many_nodes = DAE.replace("<node>", &format!("{}<node>", "<node/>".repeat(4096)));
        assert!(verify_red(&many_nodes)
            .unwrap_err()
            .to_string()
            .contains("4096 DAE"));
    }

    #[test]
    fn loader_verifies_assets_while_asset_free_validation_stays_strict() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("model.urdf");
        let mesh = dir.path().join("triangle.dae");
        let xml = urdf(RED);
        let cfg = UrdfConfig {
            motor_joints: Vec::new(),
            ..UrdfConfig::default()
        };
        std::fs::write(&path, &xml).unwrap();
        std::fs::write(&mesh, DAE).unwrap();
        assert!(Skeleton::validate_urdf(&xml, &cfg).is_err());
        let skeleton = Skeleton::try_load(&path, &cfg).unwrap();
        let expected = rerun::Asset3D::from_file_contents(
            DAE.as_bytes().to_vec(),
            Some(rerun::components::MediaType::DAE),
        );
        assert_eq!(
            skeleton.model.as_ref().unwrap().prepared_meshes[&mesh.canonicalize().unwrap()],
            expected
        );
        std::fs::write(
            &mesh,
            DAE.replace("<color>1 0 0 1</color>", "<color>0 1 0 1</color>"),
        )
        .unwrap();
        assert!(Skeleton::try_load(&path, &cfg).is_err());
        assert_eq!(
            skeleton.model.as_ref().unwrap().prepared_meshes[&mesh.canonicalize().unwrap()],
            expected
        );
    }

    #[test]
    fn shared_mesh_is_checked_against_every_links_declarations() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("model.urdf");
        std::fs::write(dir.path().join("triangle.dae"), DAE).unwrap();
        // The first link has no declaration; a later shared reference must still
        // trigger verification rather than taking an unchecked cache hit.
        let first = urdf("").replace("</robot>", "");
        let second = format!(
            r#"<link name="tip"><visual><geometry><mesh filename="./triangle.dae"/></geometry>{RED}</visual></link><joint name="fixed" type="fixed"><parent link="base"/><child link="tip"/></joint></robot>"#
        );
        let xml = format!("{first}{second}");
        let cfg = UrdfConfig {
            motor_joints: Vec::new(),
            ..UrdfConfig::default()
        };
        std::fs::write(&path, &xml).unwrap();
        let loaded = Skeleton::try_load(&path, &cfg).unwrap();
        assert_eq!(loaded.model.as_ref().unwrap().prepared_meshes.len(), 1);
        std::fs::write(&path, xml.replace("rgba=\"1 0 0 1\"", "rgba=\"0 1 0 1\"")).unwrap();
        assert!(Skeleton::try_load(&path, &cfg).is_err());
        std::fs::write(&path, urdf(RED).replace("triangle.dae", "triangle.obj")).unwrap();
        std::fs::write(
            dir.path().join("triangle.obj"),
            "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n",
        )
        .unwrap();
        assert!(Skeleton::try_load(&path, &cfg)
            .unwrap_err()
            .to_string()
            .contains("require verified embedded DAE"));
    }
}
