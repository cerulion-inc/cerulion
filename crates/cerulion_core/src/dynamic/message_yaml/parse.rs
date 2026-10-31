// SPDX-License-Identifier: AGPL-3.0-only
//! YAML text -> [`Value`], keeping integers past 64 bits.
//!
//! `serde_yaml::Value` has no integer wider than 64 bits, so reading
//! `18446744073709551616` into one fails as a parse error with no field
//! path. This reader builds the same tree as `serde_yaml::from_str::<Value>`
//! but keeps such an integer as its decimal text behind [`WIDE_INTEGER_TAG`],
//! so the decoder refuses it as out of range at its field like any other
//! integer. Past 128 bits the parser itself reads the digits as a float,
//! which the decoder refuses at its field as the wrong type.

use std::fmt;

use serde::de::{self, Deserialize, Deserializer, EnumAccess, MapAccess, SeqAccess, VariantAccess};
use serde_yaml::mapping::Entry;
use serde_yaml::value::{Tag, TaggedValue};
use serde_yaml::{Mapping, Value};

/// The tag that holds an integer past 64 bits as its decimal text. A YAML
/// tag can spell it (`!integer%20past%2064%20bits`), so [`NodeVisitor`]
/// keeps such a typed tag from ever reading as the mark.
const WIDE_INTEGER_TAG: &str = "!integer past 64 bits";

/// Parse one YAML document. Errors are the parser's own (syntax, duplicate
/// keys, more than one document).
pub(super) fn parse(text: &str) -> Result<Value, serde_yaml::Error> {
    serde_yaml::from_str::<Node>(text).map(|n| n.0)
}

/// The decimal text of an integer past 64 bits, when `v` is one.
pub(super) fn wide_integer(v: &Value) -> Option<&str> {
    match v {
        Value::Tagged(t) if t.tag == WIDE_INTEGER_TAG => match &t.value {
            Value::String(s) => Some(s),
            _ => None,
        },
        _ => None,
    }
}

fn wide(text: String) -> Value {
    Value::Tagged(Box::new(TaggedValue {
        tag: Tag::new(WIDE_INTEGER_TAG),
        value: Value::String(text),
    }))
}

/// One node of the tree, read with [`NodeVisitor`].
struct Node(Value);

impl<'de> Deserialize<'de> for Node {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        deserializer.deserialize_any(NodeVisitor).map(Node)
    }
}

/// `serde_yaml`'s own value visitor, plus the two integer widths it lacks.
struct NodeVisitor;

impl<'de> de::Visitor<'de> for NodeVisitor {
    type Value = Value;

    fn expecting(&self, f: &mut fmt::Formatter) -> fmt::Result {
        f.write_str("any YAML value")
    }

    fn visit_bool<E: de::Error>(self, b: bool) -> Result<Value, E> {
        Ok(Value::Bool(b))
    }

    fn visit_i64<E: de::Error>(self, i: i64) -> Result<Value, E> {
        Ok(Value::Number(i.into()))
    }

    fn visit_u64<E: de::Error>(self, u: u64) -> Result<Value, E> {
        Ok(Value::Number(u.into()))
    }

    fn visit_i128<E: de::Error>(self, i: i128) -> Result<Value, E> {
        Ok(wide(i.to_string()))
    }

    fn visit_u128<E: de::Error>(self, u: u128) -> Result<Value, E> {
        Ok(wide(u.to_string()))
    }

    fn visit_f64<E: de::Error>(self, f: f64) -> Result<Value, E> {
        Ok(Value::Number(f.into()))
    }

    fn visit_str<E: de::Error>(self, s: &str) -> Result<Value, E> {
        Ok(Value::String(s.to_owned()))
    }

    fn visit_string<E: de::Error>(self, s: String) -> Result<Value, E> {
        Ok(Value::String(s))
    }

    fn visit_unit<E: de::Error>(self) -> Result<Value, E> {
        Ok(Value::Null)
    }

    fn visit_none<E: de::Error>(self) -> Result<Value, E> {
        Ok(Value::Null)
    }

    fn visit_some<D: Deserializer<'de>>(self, deserializer: D) -> Result<Value, D::Error> {
        Node::deserialize(deserializer).map(|n| n.0)
    }

    fn visit_seq<A: SeqAccess<'de>>(self, mut seq: A) -> Result<Value, A::Error> {
        let mut items = Vec::with_capacity(seq.size_hint().unwrap_or(0).min(4096));
        while let Some(Node(item)) = seq.next_element()? {
            items.push(item);
        }
        Ok(Value::Sequence(items))
    }

    fn visit_map<A: MapAccess<'de>>(self, mut map: A) -> Result<Value, A::Error> {
        let mut mapping = Mapping::new();
        while let Some(Node(key)) = map.next_key()? {
            match mapping.entry(key) {
                Entry::Occupied(entry) => {
                    return Err(de::Error::custom(duplicate_key(entry.key())));
                }
                Entry::Vacant(entry) => {
                    let Node(value) = map.next_value()?;
                    entry.insert(value);
                }
            }
        }
        Ok(Value::Mapping(mapping))
    }

    fn visit_enum<A: EnumAccess<'de>>(self, data: A) -> Result<Value, A::Error> {
        let (tag, contents) = data.variant::<String>()?;
        if tag.is_empty() {
            return Err(de::Error::custom("empty YAML tag is not allowed"));
        }
        let Node(value) = contents.newtype_variant()?;
        let tag = Tag::new(tag);
        // A typed tag that spells the wide-integer mark keeps its tag, so
        // the decoder refuses it as written, but its value goes one layer
        // down: `wide_integer` needs a bare string under the mark, so the
        // input can never pass as an integer past 64 bits.
        let value = if tag == WIDE_INTEGER_TAG {
            Value::Tagged(Box::new(TaggedValue {
                tag: tag.clone(),
                value,
            }))
        } else {
            value
        };
        Ok(Value::Tagged(Box::new(TaggedValue { tag, value })))
    }
}

/// The parser's own duplicate-key text.
fn duplicate_key(key: &Value) -> String {
    match key {
        Value::Null => "duplicate entry with null key".to_string(),
        Value::Bool(b) => format!("duplicate entry with key `{b}`"),
        Value::Number(n) => format!("duplicate entry with key {n}"),
        Value::String(s) => format!("duplicate entry with key {s:?}"),
        Value::Sequence(_) | Value::Mapping(_) | Value::Tagged(_) => {
            "duplicate entry in YAML map".to_string()
        }
    }
}
