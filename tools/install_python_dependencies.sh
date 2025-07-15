#!/usr/bin/env bash
set -euo pipefail

# Get absolute path of the script's directory and go to project root
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
PROJECT_ROOT="$DIR/.."
cd "$PROJECT_ROOT"

PYTHON_VERSION="3.11.4"
PY_SHORT="$(echo "$PYTHON_VERSION" | cut -d. -f1,2)"
PYTHON_BIN="/usr/local/bin/python${PY_SHORT}"
PIP_BIN="/usr/local/bin/pip${PY_SHORT}"
TMPDIR="$PROJECT_ROOT/tmp"
mkdir -p "$TMPDIR"
export TMPDIR

# Ensure /tmp has correct permissions and try to remount if not writable
check_tmp_permissions() {
  if ! touch /tmp/test_perm 2>/dev/null; then
    chmod 1777 /tmp || true
    mount -o remount,rw,nosuid,nodev,noexec,mode=1777 /tmp || echo "  (warning: /tmp remount failed)"
  else
    rm -f /tmp/test_perm
  fi
}

# Install system packages needed to build Python and common dependencies (Linux only)
install_system_dependencies() {
  if [[ "$(uname)" == "Linux" ]]; then
    apt-get update -qq
    apt-get install -y --no-install-recommends \
      build-essential curl wget libssl-dev zlib1g-dev libbz2-dev libreadline-dev \
      libsqlite3-dev llvm libncurses-dev xz-utils tk-dev libffi-dev liblzma-dev \
      python3-openssl
  fi
}

# Download and build Python from source, then install it
build_and_install_python() {
  cd "$TMPDIR"
  if [ ! -f "Python-$PYTHON_VERSION.tgz" ]; then
    wget -q "https://www.python.org/ftp/python/$PYTHON_VERSION/Python-$PYTHON_VERSION.tgz"
  fi

  if [ ! -d "Python-$PYTHON_VERSION" ]; then
    tar xf "Python-$PYTHON_VERSION.tgz"
  fi

  cd "Python-$PYTHON_VERSION"
  ./configure --enable-optimizations --enable-shared
  make -j"$(nproc)"
  make altinstall

  if [[ "$(uname)" == "Linux" ]]; then
    echo "/usr/local/lib" > "/etc/ld.so.conf.d/python-$PYTHON_VERSION.conf"
    ldconfig
    ln -sf "$PYTHON_BIN" "/usr/bin/python${PY_SHORT}"
    ln -sf "$PIP_BIN" "/usr/bin/pip${PY_SHORT}"
  fi
}

# Create virtualenv, install Poetry, then install project dependencies
install_project_dependencies() {
  if [[ "$(uname)" == "Darwin" ]]; then
    python${PY_SHORT} -m venv .venv
  else
    "$PYTHON_BIN" -m venv .venv
  fi

  source .venv/bin/activate
  pip install --upgrade pip setuptools wheel
  pip install "poetry>=1.6,<2.0"

  poetry config virtualenvs.prefer-active-python true --local
  poetry config virtualenvs.in-project true --local
  poetry install --no-root --no-interaction

  echo "PYTHONPATH=${PWD}" > "$PROJECT_ROOT/.env"

  if [[ "$(uname)" == 'Darwin' ]]; then
    echo "# msgq doesn't work on mac" >> "$PROJECT_ROOT/.env"
    echo "export ZMQ=1" >> "$PROJECT_ROOT/.env"
    echo "export OBJC_DISABLE_INITIALIZE_FORK_SAFETY=YES" >> "$PROJECT_ROOT/.env"
  fi

  poetry self add poetry-dotenv-plugin@^0.1.0
}

# Fix permissions of Poetry's cache directory if script ran as root
fix_poetry_cache_permissions() {
  if [ -n "${SUDO_USER:-}" ] && [ "$SUDO_USER" != "root" ]; then
    POETRY_CACHE="/home/$SUDO_USER/.cache/pypoetry"
    if [ -d "$POETRY_CACHE" ]; then
      chown -R "$SUDO_USER:$SUDO_USER" "$POETRY_CACHE"
    fi
  fi
}

# Fix git submodule hook permissions if script ran as root
fix_git_hook_permissions() {
  if [ -n "${SUDO_USER:-}" ] && [ "$SUDO_USER" != "root" ]; then
    chown -R "$SUDO_USER:$SUDO_USER" "$PROJECT_ROOT/.git"
  fi
}

# Re-run script as root if on Linux and not already root
if [[ "$(uname)" == "Linux" && "$(id -u)" -ne 0 ]]; then
  exec sudo bash "$0" "$@"
fi

check_tmp_permissions
install_system_dependencies
build_and_install_python

# Run dependency setup as the original user
if [ -n "${SUDO_USER:-}" ] && [ "$SUDO_USER" != "root" ]; then
  sudo -u "$SUDO_USER" bash -c "cd '$PROJECT_ROOT' && $(declare -f install_project_dependencies); install_project_dependencies"
  fix_poetry_cache_permissions
else
  install_project_dependencies
fi

fix_git_hook_permissions

# Install pre-commit hooks (only on Linux and in a git repo)
if [[ "$(uname)" != "Darwin" && -n "${SUDO_USER:-}" && "$SUDO_USER" != "root" && -e "$PROJECT_ROOT/.git" ]]; then
  echo "pre-commit hooks install..."
  sudo -u "$SUDO_USER" bash -c "cd '$PROJECT_ROOT' && source .venv/bin/activate && poetry run pre-commit install && poetry run git submodule foreach pre-commit install || true"
fi

rm -rf "$TMPDIR"
