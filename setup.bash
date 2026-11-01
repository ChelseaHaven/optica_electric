#!/usr/bin/env bash
# Source this file to load ROS and every built workspace in this directory.
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    printf '请用 source 加载环境：\nsource %q\n' "${BASH_SOURCE[0]}" >&2
    exit 1
fi

_optica_load_environments() {
    local root workspace name
    local -A loaded=()
    root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)" || return 1
    if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
        printf '找不到 ROS 环境：/opt/ros/jazzy/setup.bash\n' >&2
        return 1
    fi
    source /opt/ros/jazzy/setup.bash || return 1

    # Load the current workspaces in stack order.
    for name in uart_ws localization_ws control_ws; do
        workspace="$root/$name"
        if [[ -f "$workspace/install/local_setup.bash" ]]; then
            source "$workspace/install/local_setup.bash" || return 1
            loaded["$workspace"]=1
        elif [[ -d "$workspace" ]]; then
            printf '跳过尚未编译的工作空间：%s\n' "$workspace" >&2
        fi
    done
    # Automatically include additional built workspaces added alongside them.
    for workspace in "$root"/*; do
        [[ -d "$workspace" ]] || continue
        [[ -z "${loaded[$workspace]:-}" ]] || continue
        [[ -f "$workspace/install/local_setup.bash" ]] || continue
        source "$workspace/install/local_setup.bash" || return 1
    done
}

if _optica_load_environments; then
    unset -f _optica_load_environments
else
    unset -f _optica_load_environments
    return 1
fi
