Import("env")
import subprocess


def get_git_commit():
    try:
        commit = subprocess.check_output(
            ["git", "rev-parse", "--short=8", "HEAD"],
            stderr=subprocess.DEVNULL,
        ).strip().decode("utf-8")
        dirty = subprocess.call(
            ["git", "diff", "--quiet"], stderr=subprocess.DEVNULL
        ) != 0
        return commit + ("-dirty" if dirty else "")
    except Exception:
        return "unknown"


env.Append(BUILD_FLAGS=['-DFIRMWARE_COMMIT=\\"%s\\"' % get_git_commit()])
