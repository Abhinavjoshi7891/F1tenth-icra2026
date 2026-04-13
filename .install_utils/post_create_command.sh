#!/bin/bash

# Setup permissions
# USER="$(id -u -n)"
sudo chown -R $USERNAME /home/$USERNAME/ws/

# Install dependencies
rosdep update &&
    rosdep install --from-paths /home/$USERNAME/ws/src --ignore-src -y

# Setup race_stack
bash ~/ws/src/race_stack/.install_utils/f110_sim_setup.sh || echo "Failed to setup f110_sim"
bash ~/ws/src/race_stack/.install_utils/gb_opt_setup.sh || echo "Failed to setup gb_opt"

# Apply Joystick patch
sudo chmod 666 /dev/input/js0
sudo chmod 666 /dev/input/event*

# setup f1tenth_gym
cd ~/ws &&
    colcon build --packages-up-to f110_gym --base-paths ~/ws \
        --cmake-args "-DCMAKE_BUILD_TYPE=Release" "-DCMAKE_EXPORT_COMPILE_COMMANDS=On" \
        -Wall -Wextra -Wpedantic --cmake-clean-cache
# ── acados ───────────────────────────────────────────────────────────────────
ACADOS_DIR=/home/$USERNAME/acados
if [ ! -f "$ACADOS_DIR/lib/libacados.so" ]; then
    echo "[post_create] Installing acados..."
    git clone https://github.com/acados/acados.git $ACADOS_DIR
    cd $ACADOS_DIR
    git checkout v0.3.2
    git submodule update --init --recursive
    mkdir -p build && cd build
    cmake .. \
        -DACADOS_WITH_QPOASES=ON \
        -DACADOS_WITH_OSQP=ON \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=$ACADOS_DIR
    make -j$(nproc)
    make install
    pip3 install -e $ACADOS_DIR/interfaces/acados_template
    echo "export ACADOS_SOURCE_DIR=$ACADOS_DIR" >> /home/$USERNAME/.bashrc_forzaeth
    echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:$ACADOS_DIR/lib" >> /home/$USERNAME/.bashrc_forzaeth
    echo "[post_create] acados installed."
else
    echo "[post_create] acados already present, skipping."
fi
