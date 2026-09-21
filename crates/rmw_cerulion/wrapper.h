// rmw_cerulion bindgen wrapper (era gating).
//
// TYPE headers only — the rmw FUNCTION surface is exported BY this crate
// (extern "C" fns in src/api/), so bindgen never needs the function
// declarations, just the struct/enum/typedef layouts they exchange.
//
// Two generation paths (see build.rs):
//   1. Against the target ROS distro's installed headers (the only
//      ABI-safe path for a deployed librmw_cerulion.so).
//   2. Against the pinned source clones for development on machines
//      without ROS (vendored bindings, layout-stamped in the file
//      header).
//
// Post-Foxy headers are include-gated on the CERULION_HAS_*_H
// defines build.rs passes after filesystem-probing the collected include
// dirs (rmw/discovery_options.h is Iron+ — an unconditional include
// aborts bindgen on Foxy AND Humble). When regenerating the VENDORED
// bindings by hand (recipe in src/ffi/vendored_bindings.rs), pass every
// define — the pinned rolling clones carry all of these headers.

#include "rmw/types.h"
#include "rmw/ret_types.h"
#include "rmw/init.h"
#include "rmw/init_options.h"
#include "rmw/security_options.h"
#include "rmw/serialized_message.h"
#include "rmw/subscription_options.h"
#include "rmw/publisher_options.h"
#include "rmw/qos_profiles.h"
#include "rmw/names_and_types.h"
#include "rmw/message_sequence.h"
#include "rmw/topic_endpoint_info.h"
#include "rmw/topic_endpoint_info_array.h"
#if defined(CERULION_HAS_SUBSCRIPTION_CONTENT_FILTER_OPTIONS_H)
// Humble+. Already reached transitively via rmw/subscription_options.h
// wherever it exists — explicit so the content-filter types cannot
// silently vanish from the bindings if upstream reshuffles includes.
#include "rmw/subscription_content_filter_options.h"
#endif
#if defined(CERULION_HAS_DISCOVERY_OPTIONS_H)
#include "rmw/discovery_options.h" // Iron+ (embedded in rmw_init_options_t)
#endif
#include "rmw/domain_id.h"
#include "rmw/event.h"
#if defined(CERULION_HAS_EVENT_CALLBACK_TYPE_H)
#include "rmw/event_callback_type.h" // Humble+
#endif
#if defined(CERULION_HAS_FEATURES_H)
#include "rmw/features.h" // Humble+
#endif
#if defined(CERULION_HAS_NETWORK_FLOW_ENDPOINT_ARRAY_H)
#include "rmw/network_flow_endpoint_array.h" // Galactic+
#endif
#if defined(CERULION_HAS_QOS_STRING_CONVERSIONS_H)
#include "rmw/qos_string_conversions.h" // Galactic+
#endif

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_runtime_c/sequence_bound.h"

#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"
#include "rosidl_typesupport_introspection_c/field_types.h"

#include "rcutils/allocator.h"
#include "rcutils/types/string_array.h"
#include "rcutils/types/uint8_array.h"
