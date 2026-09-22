// SPDX-License-Identifier: AGPL-3.0-only
//! Fail-closed preflight for explicit model loading. Legacy constructors keep
//! their compatibility behavior; this path must not silently discard geometry.
//! Imports are limited to 4096 links, 256 joint edges from the root, and
//! 4096-byte entity paths. These limits bound the legacy resolver's repeated
//! passes and stored path lengths before legacy entity-path resolution.

use std::collections::{BTreeMap, BTreeSet};

use super::{UrdfConfig, UrdfError};

const MAX_LINKS: usize = 4096;
const MAX_DEPTH: usize = 256;
const MAX_ENTITY_PATH_BYTES: usize = 4096;

pub(super) fn validate(xml: &str, cfg: &UrdfConfig) -> Result<(), UrdfError> {
    validate_config(cfg)?;
    let doc = roxmltree::Document::parse(xml).map_err(|e| UrdfError::Xml(e.to_string()))?;
    let robot = doc.root_element();
    if !robot.has_tag_name("robot") {
        return Err(UrdfError::NotRobot(robot.tag_name().name().to_string()));
    }

    let mut links = BTreeMap::new();
    for link in robot.children().filter(|n| n.has_tag_name("link")) {
        let name = required_attribute(link, "name")?;
        if links.insert(name, validate_visual(link)?).is_some() {
            return Err(invalid(link, format!("duplicate link name {name:?}")));
        }
        if links.len() > MAX_LINKS {
            return Err(invalid(
                link,
                format!("model loading supports at most {MAX_LINKS} links"),
            ));
        }
    }
    if links.is_empty() {
        return Err(UrdfError::NoLinks);
    }

    let mut joints = BTreeMap::new();
    let mut parents = BTreeMap::new();
    let mut children: BTreeMap<&str, Vec<&str>> = BTreeMap::new();
    for joint in robot.children().filter(|n| n.has_tag_name("joint")) {
        let name = required_attribute(joint, "name")?;
        let kind = required_attribute(joint, "type")?;
        let movable = match kind {
            "revolute" | "continuous" => true,
            "fixed" => false,
            _ => {
                return Err(invalid(
                    joint,
                    format!("joint {name:?} has unsupported type {kind:?}; model loading currently supports fixed, revolute, and continuous joints"),
                ));
            }
        };
        if joints.insert(name, movable).is_some() {
            return Err(invalid(joint, format!("duplicate joint name {name:?}")));
        }
        if joint.children().any(|n| n.has_tag_name("mimic")) {
            return Err(invalid(
                joint,
                "mimic joints are not supported by the current measured-motor adapter",
            ));
        }
        unique_child(joint, "origin")?;
        unique_child(joint, "axis")?;
        super::parse_origin(joint)?;
        let axis = super::joint_axis(joint)?;
        // Match the existing rotation primitive's degeneracy threshold. A tiny
        // but nonzero axis would otherwise silently render an identity rotation.
        if movable && axis.iter().map(|v| v * v).sum::<f64>().sqrt() < 1e-9 {
            return Err(invalid(
                joint,
                format!("joint {name:?} needs a nonzero motion axis with norm at least 1e-9"),
            ));
        }

        let parent = reference(joint, "parent")?;
        let child = reference(joint, "child")?;
        for endpoint in [parent, child] {
            if !links.contains_key(endpoint) {
                return Err(invalid(
                    joint,
                    format!("joint {name:?} references unknown link {endpoint:?}"),
                ));
            }
        }
        if parents.insert(child, parent).is_some() {
            return Err(invalid(
                joint,
                format!("link {child:?} has more than one parent joint"),
            ));
        }
        children.entry(parent).or_default().push(child);
    }

    for name in &cfg.motor_joints {
        match joints.get(name.as_str()) {
            Some(true) => {}
            Some(false) => {
                return Err(UrdfError::InvalidModel(format!(
                    "motor binding {name:?} must name a revolute or continuous joint"
                )))
            }
            None => {
                return Err(UrdfError::InvalidModel(format!(
                    "motor binding {name:?} does not name a joint in this model"
                )))
            }
        }
    }

    for (name, movable) in &joints {
        if *movable && !cfg.motor_joints.iter().any(|bound| bound == name) {
            return Err(UrdfError::InvalidModel(format!(
                "movable joint {name:?} has no motor binding"
            )));
        }
    }

    let roots: Vec<_> = links
        .keys()
        .filter(|name| !parents.contains_key(**name))
        .copied()
        .collect();
    if roots.len() != 1 {
        return Err(UrdfError::InvalidModel(format!(
            "expected one root link in a connected acyclic tree, found {}",
            roots.len()
        )));
    }
    let mut pending = vec![(roots[0], cfg.robot_root.clone(), 0)];
    let mut visited = BTreeSet::new();
    let mut entities = BTreeMap::new();
    while let Some((link, entity, depth)) = pending.pop() {
        if depth > MAX_DEPTH {
            return Err(UrdfError::InvalidModel(format!(
                "model loading supports at most {MAX_DEPTH} joint edges from the root"
            )));
        }
        if !visited.insert(link) {
            return Err(UrdfError::InvalidModel(format!(
                "cycle reaches link {link:?}"
            )));
        }
        reserve_entity(&mut entities, entity.clone(), format!("link {link:?}"))?;
        if links[link] {
            reserve_entity(
                &mut entities,
                format!("{entity}/mesh"),
                format!("mesh visual of link {link:?}"),
            )?;
        }
        if let Some(descendants) = children.get(link) {
            for child in descendants {
                pending.push((
                    *child,
                    format!("{entity}/{}", super::sanitize_segment(child)),
                    depth + 1,
                ));
            }
        }
    }
    if visited.len() != links.len() {
        return Err(UrdfError::InvalidModel("all links must belong to one connected acyclic tree; some links are unreachable from its root".into()));
    }
    Ok(())
}

fn validate_config(cfg: &UrdfConfig) -> Result<(), UrdfError> {
    validate_entity_length(&cfg.robot_root)?;
    if cfg.robot_root.split('/').any(|segment| {
        segment.is_empty()
            || !segment
                .chars()
                .all(|c| c.is_ascii_alphanumeric() || c == '_' || c == '-')
    }) {
        return Err(UrdfError::InvalidModel("robot_root must be a nonempty canonical entity path: slash-separated ASCII letters, digits, underscores, or hyphens, without leading or trailing slashes".into()));
    }
    if cfg.robot_root.starts_with("__") {
        return Err(UrdfError::InvalidModel(
            "robot_root must not start with '__': that root namespace is reserved by Rerun; nested segments may start with '__'".into(),
        ));
    }
    if cfg.motor_joints.len() > super::LEG_MOTOR_COUNT {
        return Err(UrdfError::InvalidModel(format!(
            "the LowState adapter supports at most {} motor bindings",
            super::LEG_MOTOR_COUNT
        )));
    }
    let mut names = BTreeSet::new();
    for name in &cfg.motor_joints {
        if name.trim().is_empty() || !names.insert(name) {
            return Err(UrdfError::InvalidModel(
                "motor_joints must contain nonempty unique joint names in measured motor order"
                    .into(),
            ));
        }
    }
    Ok(())
}

/// A link may omit visuals, but an explicit visual must be representable by the
/// renderer's one-mesh-per-link contract. Never silently skip a supplied shape.
fn validate_visual(link: roxmltree::Node<'_, '_>) -> Result<bool, UrdfError> {
    let Some(visual) = unique_child(link, "visual")? else {
        return Ok(false);
    };
    // The current renderer loads mesh assets but does not apply URDF material
    // overrides or resolve named material references. Accepting either would
    // silently lose the requested color or texture.
    if let Some(material) = visual.children().find(|n| n.has_tag_name("material")) {
        return Err(invalid(
            material,
            "URDF visual materials and textures are not supported by this loader; material import must be implemented before loading this model",
        ));
    }
    unique_child(visual, "origin")?;
    super::parse_origin(visual)?;
    let geometry = unique_child(visual, "geometry")?
        .ok_or_else(|| invalid(visual, "visual requires one mesh geometry; primitive or empty visuals are not supported by this loader"))?;
    let mut shapes = geometry.children().filter(|n| n.is_element());
    let mesh = shapes.next().filter(|n| n.has_tag_name("mesh"))
        .ok_or_else(|| invalid(geometry, "only mesh visual geometry is supported; convert the primitive to a mesh before loading"))?;
    if shapes.next().is_some() {
        return Err(invalid(
            geometry,
            "one mesh per visual is supported; combine additional geometry before loading",
        ));
    }
    required_attribute(mesh, "filename")?;
    super::vector_attribute(Some(mesh), "scale", [1.0; 3])?;
    Ok(true)
}

fn reference<'a>(joint: roxmltree::Node<'a, '_>, tag: &str) -> Result<&'a str, UrdfError> {
    let node = unique_child(joint, tag)?.ok_or_else(|| {
        invalid(
            joint,
            format!("joint requires exactly one <{tag} link=...>"),
        )
    })?;
    required_attribute(node, "link")
}

fn required_attribute<'a>(
    node: roxmltree::Node<'a, '_>,
    attribute: &str,
) -> Result<&'a str, UrdfError> {
    node.attribute(attribute)
        .filter(|value| !value.trim().is_empty())
        .ok_or_else(|| invalid(node, format!("{attribute} must be present and nonempty")))
}

fn unique_child<'a, 'input>(
    node: roxmltree::Node<'a, 'input>,
    tag: &str,
) -> Result<Option<roxmltree::Node<'a, 'input>>, UrdfError> {
    let mut matches = node.children().filter(|child| child.has_tag_name(tag));
    let first = matches.next();
    if matches.next().is_some() {
        return Err(invalid(node, format!("at most one <{tag}> is supported; combine or remove duplicate elements before loading")));
    }
    Ok(first)
}

fn invalid(node: roxmltree::Node<'_, '_>, message: impl std::fmt::Display) -> UrdfError {
    UrdfError::InvalidModel(format!(
        "<{element}> at line {line}: {message}",
        element = node.tag_name().name(),
        line = node.document().text_pos_at(node.range().start).row
    ))
}

fn reserve_entity(
    entities: &mut BTreeMap<String, String>,
    entity: String,
    owner: String,
) -> Result<(), UrdfError> {
    validate_entity_length(&entity)?;
    if let Some(previous) = entities.insert(entity.clone(), owner.clone()) {
        return Err(UrdfError::InvalidModel(format!("entity path {entity:?} collides between {previous} and {owner}; rename the link to avoid sanitized or reserved mesh paths")));
    }
    Ok(())
}

fn validate_entity_length(entity: &str) -> Result<(), UrdfError> {
    if entity.len() > MAX_ENTITY_PATH_BYTES {
        return Err(UrdfError::InvalidModel(format!(
            "model entity paths must not exceed {MAX_ENTITY_PATH_BYTES} bytes"
        )));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::skeleton::Skeleton;

    const ARM: &str = r#"<robot name="arm"><link name="base"/><link name="tip"/>
      <joint name="hinge" type="revolute"><parent link="base"/><child link="tip"/></joint></robot>"#;

    fn config(names: &[&str]) -> UrdfConfig {
        UrdfConfig {
            motor_joints: names.iter().map(|name| (*name).into()).collect(),
            ..UrdfConfig::default()
        }
    }

    fn rejects(xml: &str, cfg: &UrdfConfig, message: &str) {
        let error = validate(xml, cfg).unwrap_err().to_string();
        assert!(
            error.contains(message),
            "expected {message:?}, got {error:?}"
        );
    }

    #[test]
    fn accepts_single_root_and_measured_revolute_or_continuous_bindings() {
        assert_eq!(
            validate(r#"<robot><link name="base"/></robot>"#, &config(&[])),
            Ok(())
        );
        for kind in ["revolute", "continuous"] {
            assert_eq!(
                validate(&ARM.replace("revolute", kind), &config(&["hinge"])),
                Ok(())
            );
        }
        assert_eq!(
            validate(&ARM.replace("revolute", "fixed"), &config(&[])),
            Ok(())
        );
    }

    #[test]
    fn rejects_missing_or_duplicate_names_and_unknown_links() {
        for xml in [
            r#"<robot><link/></robot>"#,
            r#"<robot><link name="  "/></robot>"#,
        ] {
            rejects(xml, &config(&[]), "name must be present and nonempty");
        }
        rejects(
            r#"<robot><link name="a"/><link name="a"/></robot>"#,
            &config(&[]),
            "duplicate link",
        );
        rejects(
            &ARM.replace("name=\"hinge\"", "name=\"\""),
            &config(&[]),
            "name must be present and nonempty",
        );
        let duplicate = ARM.replace("</robot>", r#"<joint name="hinge" type="fixed"><parent link="tip"/><child link="base"/></joint></robot>"#);
        rejects(&duplicate, &config(&[]), "duplicate joint");
        rejects(
            &ARM.replace("child link=\"tip\"", "child link=\"absent\""),
            &config(&[]),
            "unknown link",
        );
        rejects(
            &ARM.replace("<parent link=\"base\"/>", ""),
            &config(&[]),
            "exactly one <parent",
        );
    }

    #[test]
    fn rejects_multiple_parents_disconnected_links_and_cycles() {
        let second_parent = ARM.replace("</robot>", r#"<joint name="other" type="fixed"><parent link="base"/><child link="tip"/></joint></robot>"#);
        rejects(&second_parent, &config(&[]), "more than one parent");
        rejects(
            &ARM.replace("</robot>", r#"<link name="orphan"/></robot>"#),
            &config(&["hinge"]),
            "one root link",
        );
        let cycle = r#"<robot><link name="a"/><link name="b"/><joint name="ab" type="fixed"><parent link="a"/><child link="b"/></joint><joint name="ba" type="fixed"><parent link="b"/><child link="a"/></joint></robot>"#;
        rejects(cycle, &config(&[]), "one root link");
        rejects(
            &cycle.replace("<robot>", r#"<robot><link name="root"/>"#),
            &config(&[]),
            "unreachable",
        );
    }

    #[test]
    fn rejects_unsupported_motion_and_degenerate_axes() {
        for kind in ["prismatic", "floating", "planar", "unknown"] {
            rejects(
                &ARM.replace("revolute", kind),
                &config(&[]),
                "unsupported type",
            );
        }
        for axis in ["0 0 0", "1e-12 0 0"] {
            rejects(
                &ARM.replace("</joint>", &format!(r#"<axis xyz="{axis}"/></joint>"#)),
                &config(&[]),
                "nonzero motion axis",
            );
        }
        rejects(
            &ARM.replace("</joint>", r#"<mimic joint="other"/></joint>"#),
            &config(&[]),
            "mimic joints",
        );
    }

    #[test]
    fn distinguishes_absent_values_from_invalid_or_duplicate_elements() {
        rejects(
            &ARM.replace("</joint>", r#"<origin xyz=""/></joint>"#),
            &config(&[]),
            "expected three finite numbers",
        );
        rejects(
            &ARM.replace("</joint>", r#"<axis xyz="NaN 0 0"/></joint>"#),
            &config(&[]),
            "expected three finite numbers",
        );
        for elements in [
            "<origin/><origin/>",
            "<axis/><axis/>",
            "<parent link=\"base\"/>",
        ] {
            rejects(
                &ARM.replace("</joint>", &format!("{elements}</joint>")),
                &config(&[]),
                "at most one",
            );
        }
    }

    #[test]
    fn rejects_ambiguous_or_unresolvable_motor_bindings() {
        rejects(ARM, &config(&["hinge", "hinge"]), "unique joint names");
        rejects(ARM, &config(&[""]), "nonempty unique");
        rejects(ARM, &config(&["missing"]), "does not name a joint");
        rejects(
            &ARM.replace("revolute", "fixed"),
            &config(&["hinge"]),
            "must name a revolute or continuous",
        );
        rejects(ARM, &config(&["hinge"; 13]), "at most 12 motor bindings");
    }

    #[test]
    fn rejects_empty_and_partial_bindings_for_movable_joints() {
        rejects(
            ARM,
            &config(&[]),
            "movable joint \"hinge\" has no motor binding",
        );
        let chain = r#"<robot><link name="base"/><link name="middle"/><link name="tip"/>
          <joint name="first" type="revolute"><parent link="base"/><child link="middle"/></joint>
          <joint name="second" type="continuous"><parent link="middle"/><child link="tip"/></joint></robot>"#;
        rejects(
            chain,
            &config(&["first"]),
            "movable joint \"second\" has no motor binding",
        );
        rejects(
            chain,
            &config(&["second"]),
            "movable joint \"first\" has no motor binding",
        );
        assert_eq!(validate(chain, &config(&["second", "first"])), Ok(()));
    }

    #[test]
    fn accepts_all_twelve_measured_motor_slots() {
        let mut xml = String::from(r#"<robot><link name="base"/>"#);
        let names: Vec<_> = (0..12).map(|i| format!("motor{i}")).collect();
        for name in &names {
            xml.push_str(&format!(r#"<link name="{name}"/><joint name="{name}" type="continuous"><parent link="base"/><child link="{name}"/></joint>"#));
        }
        xml.push_str("</robot>");
        let cfg = UrdfConfig {
            motor_joints: names,
            ..config(&[])
        };
        assert_eq!(validate(&xml, &cfg), Ok(()));
    }

    #[test]
    fn bounds_model_size_and_depth_before_the_legacy_resolver() {
        for (links, chain, error) in [
            (4096, false, None),
            (4097, false, Some("at most 4096 links")),
            (257, true, None),
            (258, true, Some("at most 256 joint edges")),
        ] {
            let mut xml = String::from(r#"<robot><link name="link0"/>"#);
            for child in 1..links {
                let parent = if chain { child - 1 } else { 0 };
                xml.push_str(&format!(r#"<link name="link{child}"/><joint name="joint{child}" type="fixed"><parent link="link{parent}"/><child link="link{child}"/></joint>"#));
            }
            xml.push_str("</robot>");
            if let Some(message) = error {
                rejects(&xml, &config(&[]), message);
            } else {
                assert_eq!(validate(&xml, &config(&[])), Ok(()));
            }
        }
    }

    #[test]
    fn bounds_entity_paths_including_mesh_children() {
        let cfg = UrdfConfig {
            robot_root: "r".repeat(4096),
            ..config(&[])
        };
        assert_eq!(
            validate(r#"<robot><link name="base"/></robot>"#, &cfg),
            Ok(())
        );
        rejects(
            &ARM.replace("revolute", "fixed"),
            &cfg,
            "must not exceed 4096 bytes",
        );
        rejects(
            r#"<robot><link name="base"><visual><geometry><mesh filename="body.glb"/></geometry></visual></link></robot>"#,
            &cfg,
            "must not exceed 4096 bytes",
        );
    }

    #[test]
    fn rejects_reserved_entity_roots_but_allows_nested_double_underscores() {
        for root in ["__", "__robot", "__properties", "__robot/path"] {
            let cfg = UrdfConfig {
                robot_root: root.into(),
                ..config(&[])
            };
            let error =
                Skeleton::validate_urdf(&ARM.replace("revolute", "fixed"), &cfg).unwrap_err();
            assert!(matches!(error, UrdfError::InvalidModel(_)));
            assert!(error.to_string().contains("reserved"), "{error}");
        }
        for root in ["_robot", "world/__nested", "world/__properties"] {
            let cfg = UrdfConfig {
                robot_root: root.into(),
                ..config(&[])
            };
            assert_eq!(
                Skeleton::validate_urdf(&ARM.replace("revolute", "fixed"), &cfg),
                Ok(()),
                "{root}"
            );
        }
    }

    #[test]
    fn rejects_noncanonical_entity_roots() {
        for root in [
            "",
            "/robot",
            "robot/",
            "robot//arm",
            "robot/../arm",
            "robot/./arm",
            "robot\\arm",
            "robot arm",
        ] {
            let cfg = UrdfConfig {
                robot_root: root.into(),
                ..config(&[])
            };
            rejects(ARM, &cfg, "canonical entity path");
        }
    }

    #[test]
    fn rejects_sibling_sanitizer_aliases_using_a_handwritten_hash_oracle() {
        // The shared sanitizer's pinned vector is a/b -> a_b_cf61. An already
        // clean sibling with that spelling aliases it without any hash search.
        let xml = r#"<robot><link name="root"/><link name="a/b"/><link name="a_b_cf61"/>
          <joint name="one" type="fixed"><parent link="root"/><child link="a/b"/></joint>
          <joint name="two" type="fixed"><parent link="root"/><child link="a_b_cf61"/></joint></robot>"#;
        rejects(xml, &config(&[]), "entity path");
        // The same sanitized segment at a different depth is not a collision.
        let chain = xml.replace(
            r#"<parent link="root"/><child link="a_b_cf61"/>"#,
            r#"<parent link="a/b"/><child link="a_b_cf61"/>"#,
        );
        assert_eq!(validate(&chain, &config(&[])), Ok(()));
    }

    #[test]
    fn rejects_mesh_child_collisions_but_allows_unreserved_mesh_names() {
        let bare = ARM.replace("revolute", "fixed").replace("tip", "mesh");
        assert_eq!(validate(&bare, &config(&[])), Ok(()));
        let mesh_parent = bare.replace(r#"<link name="base"/>"#, r#"<link name="base"><visual><geometry><mesh filename="body.glb"/></geometry></visual></link>"#);
        rejects(&mesh_parent, &config(&[]), "reserved mesh paths");
    }

    #[test]
    fn rejects_visuals_that_the_renderer_would_silently_skip() {
        for (visual, message) in [
            ("<visual/>", "visual requires one mesh"),
            (
                r#"<visual><geometry><box size="1 1 1"/></geometry></visual>"#,
                "only mesh visual",
            ),
            (
                r#"<visual><geometry><mesh filename=""/></geometry></visual>"#,
                "filename must be present",
            ),
            (
                r#"<visual><geometry><mesh filename="a.glb"/><mesh filename="b.glb"/></geometry></visual>"#,
                "one mesh per visual",
            ),
            (
                r#"<visual><geometry><mesh filename="a.glb"/></geometry></visual><visual><geometry><mesh filename="b.glb"/></geometry></visual>"#,
                "at most one <visual>",
            ),
            (
                r#"<visual><geometry><mesh filename="a.glb" scale="1 2"/></geometry></visual>"#,
                "expected three finite numbers",
            ),
        ] {
            rejects(
                &format!(r#"<robot><link name="base">{visual}</link></robot>"#),
                &config(&[]),
                message,
            );
        }
        assert_eq!(
            validate(
                r#"<robot><link name="base"><visual><geometry><mesh filename="body.glb"/></geometry></visual></link></robot>"#,
                &config(&[])
            ),
            Ok(())
        );
    }

    #[test]
    fn rejects_visual_material_colors_textures_and_named_references() {
        for (definition, material) in [
            (
                "",
                r#"<material name="red"><color rgba="1 0 0 1"/></material>"#,
            ),
            (
                "",
                r#"<material name="textured"><texture filename="body.png"/></material>"#,
            ),
            (
                r#"<material name="paint"><color rgba="0 1 0 1"/></material>"#,
                r#"<material name="paint"/>"#,
            ),
            ("", r#"<material name="undefined"/>"#),
        ] {
            let xml = format!(
                "<robot>{definition}\n<link name=\"base\"><visual><geometry><mesh filename=\"body.glb\"/></geometry>\n{material}\n</visual></link></robot>"
            );
            let error = validate(&xml, &config(&[])).unwrap_err();
            assert!(matches!(error, UrdfError::InvalidModel(_)));
            let message = error.to_string();
            assert!(message.contains("<material> at line 3"), "{message}");
            assert!(
                message.contains("visual materials and textures are not supported"),
                "{message}"
            );
        }
    }

    #[test]
    fn unused_material_declarations_do_not_override_mesh_appearance() {
        let xml = r#"<robot><material name="unused"><color rgba="1 0 0 1"/></material>
          <link name="base"><visual><geometry><mesh filename="body.glb"/></geometry></visual></link></robot>"#;
        assert_eq!(validate(xml, &config(&[])), Ok(()));
    }

    #[test]
    fn malformed_xml_and_missing_robot_links_keep_their_error_types() {
        assert!(matches!(
            validate("<robot>", &config(&[])),
            Err(UrdfError::Xml(_))
        ));
        assert_eq!(
            validate("<other/>", &config(&[])),
            Err(UrdfError::NotRobot("other".into()))
        );
        assert_eq!(validate("<robot/>", &config(&[])), Err(UrdfError::NoLinks));
    }
}
