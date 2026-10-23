#!/usr/bin/env bash
set -e
echo $$ > "${1:?Pass a PID file inside the local session directory}"
exec sleep 3600
