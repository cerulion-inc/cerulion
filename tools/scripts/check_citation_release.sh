#!/usr/bin/env bash

set -euo pipefail

if [ "$#" -lt 2 ] || [ "$#" -gt 3 ]; then
    printf 'usage: %s <version> <tag-ref-name> [citation-file]\n' "$0" >&2
    exit 2
fi

version=$1
tag_ref_name=$2
citation_file=${3:-CITATION.cff}

cff_version=$(sed -En 's/^version:[[:space:]]*"?([^"]*)"?[[:space:]]*$/\1/p' "$citation_file" | sed -n '1p')
if [ "$cff_version" != "$version" ]; then
    echo "::error::CITATION.cff version ${cff_version} does not match workspace version ${version}; update CITATION.cff"
    exit 1
fi

if [ -z "$tag_ref_name" ]; then
    exit 0
fi

tag_date="${CITATION_TAG_DATE:-$(TZ=UTC git log -1 --format=%cd --date=format-local:%Y-%m-%d)}"
run_date="${CITATION_RUN_DATE:-$(date -u +%Y-%m-%d)}"
cff_date=$(sed -En 's/^date-released:[[:space:]]*"?([^"]*)"?[[:space:]]*$/\1/p' "$citation_file" | sed -n '1p')
if [ -z "$cff_date" ]; then
    echo "::error::CITATION.cff date-released is missing; expected a date in [${tag_date}, ${run_date}], found <missing>; edit CITATION.cff to add date-released: \"${run_date}\""
    exit 1
fi
if [[ ! "$cff_date" =~ ^[0-9]{4}-[0-9]{2}-[0-9]{2}$ ]]; then
    echo "::error::CITATION.cff date-released is malformed: found ${cff_date}; expected YYYY-MM-DD in [${tag_date}, ${run_date}]"
    exit 1
fi
canonical_date=
if ! canonical_date=$(python3 -c 'import datetime, sys; print(datetime.date.fromisoformat(sys.argv[1]).isoformat())' "$cff_date" 2>/dev/null); then
    echo "::error::CITATION.cff date-released is not a real calendar date: found ${cff_date}; expected a valid YYYY-MM-DD date in [${tag_date}, ${run_date}]"
    exit 1
fi
if [ "$canonical_date" != "$cff_date" ]; then
    echo "::error::CITATION.cff date-released is not a real calendar date: found ${cff_date}; expected a valid YYYY-MM-DD date in [${tag_date}, ${run_date}]"
    exit 1
fi
if [[ "$cff_date" < "$tag_date" ]]; then
    echo "::error::CITATION.cff date-released ${cff_date} predates the tagged commit ${tag_date}; set date-released to a date in [${tag_date}, ${run_date}]"
    exit 1
fi
if [[ "$cff_date" > "$run_date" ]]; then
    echo "::error::CITATION.cff date-released ${cff_date} is in the future relative to the release run date ${run_date}; set date-released to a date in [${tag_date}, ${run_date}]"
    exit 1
fi
echo "citation gate accepted: tag=${tag_ref_name} version=${version} tag_date=${tag_date} run_date=${run_date} cff_date=${cff_date}"
