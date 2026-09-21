#!/bin/sh

set -eu

manifest=${1:-Cargo.toml}
[ -f "$manifest" ] || {
    printf 'error: could not parse [workspace.package].version from %s\n' "$manifest" >&2
    exit 1
}

# shellcheck disable=SC2016
version=$(LC_ALL=C "${CERULION_WORKSPACE_VERSION_AWK:-awk}" '
    function scan_delimiters(text,    i, character, quote, escaped) {
        delimiter_open = 0
        delimiter_value = ""
        quote = ""
        escaped = 0
        for (i = 1; i <= length(text); i++) {
            character = substr(text, i, 1)
            if (quote == "") {
                if (character == "#") {
                    break
                }
                if (character == "\"" || character == "\047") {
                    if (substr(text, i, 3) == "\"\"\"" ||
                        substr(text, i, 3) == "\047\047\047") {
                        delimiter_open = 1
                        delimiter_value = substr(text, i, 3)
                        quote = "multiline"
                        i += 2
                    } else {
                        quote = character
                    }
                }
            } else if (quote == "multiline") {
                if (delimiter_value == "\"\"\"") {
                    if (escaped) {
                        escaped = 0
                        continue
                    }
                    if (character == "\\") {
                        escaped = 1
                        continue
                    }
                }
                if (substr(text, i, 3) == delimiter_value) {
                    delimiter_open = 0
                    delimiter_value = ""
                    quote = ""
                    i += 2
                }
            } else if (quote == "\"") {
                if (escaped) {
                    escaped = 0
                } else if (character == "\\") {
                    escaped = 1
                } else if (character == "\"") {
                    quote = ""
                }
            } else if (character == "\047") {
                quote = ""
            }
        }
    }

    function multiline_closes(text, delimiter,    i, character, escaped) {
        escaped = 0
        for (i = 1; i <= length(text); i++) {
            character = substr(text, i, 1)
            if (delimiter == "\"\"\"") {
                if (escaped) {
                    escaped = 0
                    continue
                }
                if (character == "\\") {
                    escaped = 1
                    continue
                }
            }
            if (substr(text, i, 3) == delimiter) {
                return 1
            }
        }
        return 0
    }

    function hex_value(text,    i, digit, result) {
        result = 0
        for (i = 1; i <= length(text); i++) {
            digit = tolower(substr(text, i, 1))
            result *= 16
            result += index("0123456789abcdef", digit) - 1
        }
        return result
    }

    function unicode_scalar(codepoint,    byte1, byte2, byte3, byte4) {
        if (codepoint <= 127) {
            return sprintf("%c", codepoint)
        }
        if (codepoint <= 2047) {
            byte1 = 192 + int(codepoint / 64)
            byte2 = 128 + (codepoint % 64)
            return sprintf("%c%c", byte1, byte2)
        }
        if (codepoint <= 65535) {
            byte1 = 224 + int(codepoint / 4096)
            byte2 = 128 + (int(codepoint / 64) % 64)
            byte3 = 128 + (codepoint % 64)
            return sprintf("%c%c%c", byte1, byte2, byte3)
        }
        byte1 = 240 + int(codepoint / 262144)
        byte2 = 128 + (int(codepoint / 4096) % 64)
        byte3 = 128 + (int(codepoint / 64) % 64)
        byte4 = 128 + (codepoint % 64)
        return sprintf("%c%c%c%c", byte1, byte2, byte3, byte4)
    }

    function decode_basic(text,    i, character, escaped, result, escape_code,
                          width, digits, codepoint) {
        result = ""
        escaped = 0
        for (i = 1; i <= length(text); i++) {
            character = substr(text, i, 1)
            if (!escaped) {
                if (character == "\\") {
                    escaped = 1
                } else if (character == "\"") {
                    print "error: unescaped quote in [workspace.package].version" > "/dev/stderr"
                    exit 1
                } else {
                    result = result character
                }
                continue
            }
            escape_code = character
            if (escape_code == "b") result = result "\b"
            else if (escape_code == "t") result = result "\t"
            else if (escape_code == "n") result = result "\n"
            else if (escape_code == "f") result = result "\f"
            else if (escape_code == "r") result = result "\r"
            else if (escape_code == "\"" || escape_code == "\\") result = result escape_code
            else if (escape_code == "u" || escape_code == "U") {
                width = (escape_code == "u") ? 4 : 8
                digits = substr(text, i + 1, width)
                if (length(digits) != width ||
                    (width == 4 &&
                     digits !~ /^[[:xdigit:]][[:xdigit:]][[:xdigit:]][[:xdigit:]]$/) ||
                    (width == 8 &&
                     digits !~ /^[[:xdigit:]][[:xdigit:]][[:xdigit:]][[:xdigit:]][[:xdigit:]][[:xdigit:]][[:xdigit:]][[:xdigit:]]$/)) {
                    print "error: malformed Unicode escape in [workspace.package].version" > "/dev/stderr"
                    exit 1
                }
                codepoint = hex_value(digits)
                if (codepoint > 1114111 ||
                    (codepoint >= 55296 && codepoint <= 57343)) {
                    print "error: invalid Unicode scalar in [workspace.package].version" > "/dev/stderr"
                    exit 1
                }
                result = result unicode_scalar(codepoint)
                i += width
            }
            else {
                print "error: invalid escape in [workspace.package].version" > "/dev/stderr"
                exit 1
            }
            escaped = 0
        }
        if (escaped) {
            print "error: unterminated escape in [workspace.package].version" > "/dev/stderr"
            exit 1
        }
        return result
    }

    function normalize_header(text,    i, character, escaped, quote, result) {
        result = ""
        quote = ""
        escaped = 0
        for (i = 1; i <= length(text); i++) {
            character = substr(text, i, 1)
            if (quote == "") {
                if (character ~ /[[:space:]]/) {
                    continue
                }
                if (character == "\"" || character == "\047") {
                    quote = character
                }
            } else if (quote == "\"") {
                if (escaped) {
                    escaped = 0
                } else if (character == "\\") {
                    escaped = 1
                } else if (character == "\"") {
                    quote = ""
                }
            } else if (character == "\047") {
                quote = ""
            }
            result = result character
        }
        return result
    }

    {
        line = $0
        if (multiline_delimiter != "") {
            if (multiline_closes(line, multiline_delimiter)) {
                multiline_delimiter = ""
            }
            next
        }
        scan_delimiters(line)
        if (delimiter_open && delimiter_value != "") {
            multiline_delimiter = delimiter_value
            next
        }
        header_line = line
        sub(/[[:space:]]*#.*/, "", header_line)
        header_line = normalize_header(header_line)
        if (header_line ~ /^\[(workspace|"workspace"|\047workspace\047)\.(package|"package"|\047package\047)\]$/) {
            in_workspace_package = 1
            next
        }
        if (!in_workspace_package) {
            next
        }
        version_line = line
        sub(/[[:space:]]*#.*/, "", version_line)
        if (version_line ~ /^[[:space:]]*version[[:space:]]*=/) {
            sub(/^[[:space:]]*version[[:space:]]*=[[:space:]]*/, "", version_line)
            sub(/[[:space:]]*$/, "", version_line)
            if (version_line ~ /^"""|^\047\047\047/) {
                print "error: [workspace.package].version must not be a multiline string" > "/dev/stderr"
                exit 1
            }
        }
        if (line ~ /^[[:space:]]*\[/) { exit }
        sub(/[[:space:]]*#.*/, "", line)
        if (line ~ /^[[:space:]]*version[[:space:]]*=/) {
            sub(/^[[:space:]]*version[[:space:]]*=[[:space:]]*/, "", line)
            sub(/[[:space:]]*$/, "", line)
            if (substr(line, 1, 1) == "\"" &&
                substr(line, length(line), 1) == "\"") {
                print decode_basic(substr(line, 2, length(line) - 2))
                exit
            }
            if (line ~ /^\047[^\047]+\047$/) {
                sub(/^\047/, "", line)
                sub(/\047$/, "", line)
                print line
                exit
            }
        }
    }
' < "$manifest") || version=

[ -n "$version" ] || {
    printf 'error: could not parse [workspace.package].version from %s\n' "$manifest" >&2
    exit 1
}

printf '%s\n' "$version"
