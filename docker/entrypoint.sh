#!/usr/bin/env bash
set -e

TARGET_DIR="$SCRIPT_DIR/../"
MARKER="$HOME/.oceansim_initialized"
echo "[entrypoint] starting"

sudo ldconfig || true

# First-run initialization only
if [ ! -f "$MARKER" ]; then
    echo "[entrypoint] first run detected"


    if [ ! -d "$TARGET_DIR/built_isaacsim" ]; then # redundant
    # Move the built sim to the host<->docker directory bind
        echo "[entrypoint] copying isaacsim"
        sudo cp -a "$HOME/isaacsim/_build/linux-x86_64/release" "$TARGET_DIR/built_isaacsim"
        # Remove the isaacsim source files 
        #sudo rm -R "$HOME/isaacsim"
    #sudo mv /home/efandi/isaacsim /home/efandi/isaacsim_src
        sudo chown -R $(id -u):$(id -g) "$TARGET_DIR"
# Adding a symlink for easier launching (actually a shell script to launch in correct dir)
#     cat <<EOF > "$TARGET_DIR/run_isaac.sh"
#     #!/bin/bash
#     cd "$TARGET_DIR/isaacsim/_build/linux-x86_64/release"
#     ./isaac-sim.sh "\$@"
# EOF
#         chmod +x "$TARGET_DIR/run_isaac.sh"
    else
        echo "[entrypoint] Unable to add isaacsim to shared directory - did you delete your existing shared directory isaacsim/ folder?"
    fi

    touch "$MARKER"
    echo "[entrypoint] initialization complete"
else
    echo "[entrypoint] already initialized, skipping setup"
fi

exec "$@"
