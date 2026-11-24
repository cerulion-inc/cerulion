#!/bin/sh
# Render a figure the way a GitHub README shows it, in the light and the dark theme: the figure
# as an <img> in a README-width column on the theme's page colour, at 2x, into
# docs/media/charts/out/renders/<name>.github-{light,dark}.png. Each theme shows the figure's
# twin for that theme, <name>-light.svg or <name>-dark.svg (the file the README's <picture> block
# picks), or <name>.svg when the figure has no twins.
#
#   docs/media/charts/render-themes.sh native-rtt platform-rtt harness-results harness-success-rate harness-cost-per-success
#
# It reads the SVGs in docs/media/ and needs Google Chrome on macOS. Nothing it writes is
# tracked: out/ is ignored by git.
set -e
cd "$(dirname "$0")"
mkdir -p out/renders
CHROME="/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"
for name in "${@:-native-rtt}"; do
  [ -f "../$name.svg" ] || { echo "no such figure: docs/media/$name.svg" >&2; exit 1; }
  for theme in light dark; do
    if [ "$theme" = dark ]; then bg="#0d1117"; fg="#f0f6fc"; else bg="#ffffff"; fg="#1f2328"; fi
    file="$name.svg"; [ -f "../$name-$theme.svg" ] && file="$name-$theme.svg"
    page="out/renders/.$name.$theme.html"
    cat > "$page" <<HTML
<!doctype html><meta charset="utf-8">
<body style="margin:0;background:$bg;color:$fg;font:16px -apple-system,BlinkMacSystemFont,'Segoe UI',Helvetica,Arial,sans-serif">
<div style="width:894px;margin:0 auto;padding:32px 45px">
<h2 style="border-bottom:1px solid #3d444d55;padding-bottom:.3em">Performance</h2>
<p><img src="../../../$file" alt="$name" style="max-width:100%"></p>
<p>README text under the figure, in the $theme theme.</p>
</div></body>
HTML
    "$CHROME" --headless=new --disable-gpu --hide-scrollbars --force-device-scale-factor=2 \
      --window-size=984,1400 --screenshot="out/renders/$name.github-$theme.png" "file://$PWD/$page" 2>/dev/null
    rm -f "$page"
    echo "rendered $name in the $theme theme"
  done
done
