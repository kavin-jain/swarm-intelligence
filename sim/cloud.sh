#!/usr/bin/env bash
# Paired benchmark on GitHub Actions (.github/workflows/bench.yml), so the laptop stays free.
#   sim/cloud.sh LABEL DEV_REF BASE_REF ["DEV_ENV"] ["BASE_ENV"] [capacity]
# e.g. sim/cloud.sh metering HEAD HEAD "SIM_DENSITY=3" "" true    (one version, two settings, with the capacity sweep)
# Both refs must be pushed. Prints the report and keeps it in build/cloud/LABEL/report.md.
set -euo pipefail
label=$1; dev=$(git rev-parse "$2"); base=$(git rev-parse "$3")
gh workflow run bench.yml -f label="$label" -f dev="$dev" -f base="$base" -f dev_env="${4:-}" -f base_env="${5:-}" -f capacity="${6:-false}"
id=""
for _ in $(seq 30); do
    id=$(gh run list --workflow bench.yml --limit 20 --json databaseId,displayTitle -q ".[] | select(.displayTitle == \"bench $label\") | .databaseId" | head -1)
    [ -n "$id" ] && break
    sleep 5
done
[ -n "$id" ] || { echo "run 'bench $label' did not appear" >&2; exit 1; }
echo "run $id: $(gh run view "$id" --json url -q .url)"
gh run watch "$id" --interval 30 > /dev/null || true
mkdir -p "build/cloud/$label"
gh run download "$id" -n report -D "build/cloud/$label"
cat "build/cloud/$label/report.md"
