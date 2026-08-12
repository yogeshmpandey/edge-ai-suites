#!/usr/bin/env bash
# SPDX-FileCopyrightText: (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
#
# Parse resources/broker-config/intersections.yaml without external tools.
#
# Usage: parse-intersections.sh <intersections.yaml> [default_rtsp_port]
#
# Emits one normalised, pipe-separated record per line:
#   I|<id>|<name>|<ip>|<mqtt_port>|<enabled>
#   C|<id>|<camera_name>|<camera_url>
#
# Each camera's RTSP source is a self-contained URL. When a camera omits
# ``url``, one is derived as rtsp://<intersection_ip>:<default_rtsp_port>/<path>
# (path = the camera name with its "<id>-" prefix stripped) as a same-host
# convenience default — cameras are expected to commonly live on different
# hosts, so there is no shared per-intersection ip/port to fall back to beyond
# this.
#
# Exit codes: 0 ok, 1 file missing/unreadable, 2 no usable intersections,
#             3 invalid entry (missing ip).

set -euo pipefail

CONFIG_FILE="${1:-}"
DEFAULT_RTSP_PORT="${2:-8554}"

if [[ -z "${CONFIG_FILE}" ]]; then
    echo "usage: $(basename "$0") <intersections.yaml> [default_rtsp_port]" >&2
    exit 1
fi

if [[ ! -r "${CONFIG_FILE}" ]]; then
    echo "intersections config not readable: ${CONFIG_FILE}" >&2
    exit 1
fi

output=$(awk -v default_port="${DEFAULT_RTSP_PORT}" '
function ltrim(s) { sub(/^[ \t]+/, "", s); return s }
function rtrim(s) { sub(/[ \t]+$/, "", s); return s }
function clean(s) {
    sub(/[ \t]+#.*$/, "", s)
    s = rtrim(ltrim(s))
    if (s ~ /^".*"$/ || s ~ /^'"'"'.*'"'"'$/) { s = substr(s, 2, length(s) - 2) }
    if (s == "null" || s == "~") { s = "" }
    return s
}
function setval(key, value) {
    if (scope == "cam") {
        if (key == "name") cam_name[ni, ci] = value
        else if (key == "url") cam_url[ni, ci] = value
    } else if (scope == "int") {
        if (key == "id") int_id[ni] = value
        else if (key == "name") int_name[ni] = value
        else if (key == "ip") int_ip[ni] = value
        else if (key == "mqtt_port") int_mqtt[ni] = value
        else if (key == "enabled") int_enabled[ni] = value
    }
}
{
    line = $0
    sub(/\r$/, "", line)
    body = ltrim(line)
    if (body == "" || body ~ /^#/) next
    indent = length(line) - length(body)

    if (body ~ /^intersections:/) { in_root = 1; next }
    if (!in_root) next

    if (body ~ /^-/) {
        body = ltrim(substr(body, 2))
        # camera items may be indented deeper than "cameras:" or aligned with it
        if (cam_mode && indent >= cam_indent) {
            scope = "cam"; ci = ++ncam[ni]
        } else {
            cam_mode = 0; scope = "int"; ni++; ncam[ni] = 0; ci = 0
        }
        if (body == "") next
    } else {
        if (body ~ /^cameras:/) { cam_mode = 1; cam_indent = indent; next }
        if (cam_mode && scope == "cam" && indent <= cam_indent) { cam_mode = 0; scope = "int" }
    }

    pos = index(body, ":")
    if (pos == 0) next
    key = clean(substr(body, 1, pos - 1))
    value = clean(substr(body, pos + 1))
    if (key == "cameras") { cam_mode = 1; cam_indent = indent; next }
    if (value == "") next
    setval(key, value)
}
END {
    for (n = 1; n <= ni; n++) {
        id = int_id[n]
        if (id == "") {
            for (c = 1; c <= ncam[n]; c++) {
                nm = cam_name[n, c]
                p = index(nm, "-")
                if (p > 1 && p < length(nm)) { id = substr(nm, 1, p - 1); break }
            }
        }
        if (id == "") { print "intersection #" n ": missing id and camera names" > "/dev/stderr"; err = 1; continue }
        if (int_ip[n] == "") { print "intersection " id ": missing ip" > "/dev/stderr"; err = 1; continue }

        name = (int_name[n] != "") ? int_name[n] : id
        mqtt = (int_mqtt[n] != "") ? int_mqtt[n] : 1883
        enabled = (int_enabled[n] != "") ? tolower(int_enabled[n]) : "true"
        print "I|" id "|" name "|" int_ip[n] "|" mqtt "|" enabled

        if (ncam[n] == 0) {
            for (c = 1; c <= 4; c++) {
                curl = "rtsp://" int_ip[n] ":" default_port "/camera" c
                print "C|" id "|" id "-camera" c "|" curl
            }
            continue
        }
        for (c = 1; c <= ncam[n]; c++) {
            nm = (cam_name[n, c] != "") ? cam_name[n, c] : (id "-camera" c)
            curl = cam_url[n, c]
            if (curl == "") {
                cpath = nm
                sub(/^[^-]*-/, "", cpath)
                curl = "rtsp://" int_ip[n] ":" default_port "/" cpath
            }
            print "C|" id "|" nm "|" curl
        }
    }
    exit err ? 3 : 0
}
' "${CONFIG_FILE}") || exit 3

if [[ -z "${output}" ]]; then
    exit 2
fi

printf '%s\n' "${output}"
