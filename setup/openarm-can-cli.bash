# /etc/bash_completion.d/openarm-can-cli
_openarm_can_cli()
{
    local cur prev words cword
    _init_completion || return

    local subcommands="can_configure discover show_param write_param set_zero change_id change_baud enable disable clear_error monitor"

    local subcommand=""
    for word in "${words[@]}"; do
        case "${word}" in
            can_configure|discover|show_param|write_param|set_zero|change_id|change_baud|enable|disable|clear_error|monitor)
                subcommand="${word}"
                break
                ;;
        esac
    done

    case "${prev}" in
        -i|--interface)
            COMPREPLY=($(compgen -W "$(ls /sys/class/net | grep can)" -- "${cur}"))
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
            enable|disable|clear_error|monitor|show_param|set_zero)
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