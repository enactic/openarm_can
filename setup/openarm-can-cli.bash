# Copyright 2026 Enactic, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# shellcheck shell=bash
#
# Disable SC2207: "Prefer mapfile or read -a to split command output (or quote to avoid splitting)."
# Completion candidates are static option/subcommand names with no spaces or glob characters,
# so word splitting is safe here.
# shellcheck disable=SC2207

_openarm_can_cli() {
    local cur prev words
    _init_completion || return

    local subcommands="can_configure discover show_param write_param set_zero change_id change_baud enable disable clear_error monitor"

    local subcommand=""
    for word in "${words[@]}"; do
        case "${word}" in
        can_configure | discover | show_param | write_param | set_zero | change_id | change_baud | enable | disable | clear_error | monitor)
            subcommand="${word}"
            break
            ;;
        esac
    done

    case "${prev}" in
    -i | --interface)
        local ifaces=()
        for iface in /sys/class/net/can*; do
            [[ -e "${iface}" ]] && ifaces+=("${iface##*/}")
        done
        COMPREPLY=($(compgen -W "${ifaces[*]}" -- "${cur}"))
        return
        ;;
    esac

    if [[ -n "${subcommand}" ]]; then
        case "${subcommand}" in
        can_configure)
            COMPREPLY=($(compgen -W "-b --bitrate -d --dbitrate --sp --dsp --dsjw --rm --no-fd" -- "${cur}"))
            ;;
        discover)
            COMPREPLY=($(compgen -W "-m --max-id --full-scan" -- "${cur}"))
            ;;
        enable | disable | clear_error | monitor | show_param | set_zero)
            COMPREPLY=($(compgen -W "-a --arm --no-arm --id" -- "${cur}"))
            ;;
        change_id)
            COMPREPLY=($(compgen -W "-c --current -s --new-slave -m --new-master --save" -- "${cur}"))
            ;;
        change_baud)
            COMPREPLY=($(compgen -W "-b --baudrate -c --canid --save" -- "${cur}"))
            ;;
        write_param)
            COMPREPLY=($(compgen -W "-c --id -r --rid -v --value --save" -- "${cur}"))
            ;;
        esac
        return
    fi

    if [[ "${cur}" == -* ]]; then
        COMPREPLY=($(compgen -W "-i --interface -h --help" -- "${cur}"))
    else
        COMPREPLY=($(compgen -W "${subcommands}" -- "${cur}"))
    fi
}

complete -F _openarm_can_cli openarm-can-cli
