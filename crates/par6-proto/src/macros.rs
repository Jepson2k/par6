//! Crate-internal macros.

/// Defines a wire enum plus the reflection needed by codegen and decode:
/// `ALL`, `variants()` (name/value pairs) and `from_wire()`.
///
/// Keeping enum definition and reflection in one macro is what guarantees the
/// generated Python mirror can never drift from the Rust enums.
macro_rules! wire_enum {
    (
        $(#[$meta:meta])*
        $name:ident: $repr:ty {
            $($(#[$vmeta:meta])* $variant:ident = $value:literal,)+
        }
    ) => {
        $(#[$meta])*
        #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
        #[repr($repr)]
        pub enum $name {
            $($(#[$vmeta])* $variant = $value,)+
        }

        impl $name {
            /// Every variant, in declaration order.
            pub const ALL: &'static [$name] = &[$($name::$variant,)+];

            /// `(rust_name, wire_value)` pairs, in declaration order.
            pub fn variants() -> &'static [(&'static str, i64)] {
                &[$((stringify!($variant), $value),)+]
            }

            /// Map a wire integer back to the enum, if it names a variant.
            pub fn from_wire(v: i64) -> Option<Self> {
                $(if v == $value {
                    return Some($name::$variant);
                })+
                None
            }
        }

        // Deserialized from the wire integer OR the variant name
        // (case-insensitive): the Python binding sends the numbers it puts
        // on the socket, while a script spells `enter_flashing("parked")`.
        impl<'de> serde::Deserialize<'de> for $name {
            fn deserialize<D: serde::Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
                struct V;
                impl serde::de::Visitor<'_> for V {
                    type Value = $name;
                    fn expecting(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
                        write!(f, "a {} wire value or variant name", stringify!($name))
                    }
                    fn visit_i64<E: serde::de::Error>(self, v: i64) -> Result<$name, E> {
                        $name::from_wire(v)
                            .ok_or_else(|| E::custom(format!("{v} is not a {}", stringify!($name))))
                    }
                    fn visit_u64<E: serde::de::Error>(self, v: u64) -> Result<$name, E> {
                        self.visit_i64(i64::try_from(v).map_err(E::custom)?)
                    }
                    fn visit_str<E: serde::de::Error>(self, v: &str) -> Result<$name, E> {
                        $name::variants()
                            .iter()
                            .find(|(n, _)| n.eq_ignore_ascii_case(v))
                            .and_then(|(_, w)| $name::from_wire(*w))
                            .ok_or_else(|| E::custom(format!("{v:?} is not a {}", stringify!($name))))
                    }
                }
                d.deserialize_any(V)
            }
        }
    };
}
