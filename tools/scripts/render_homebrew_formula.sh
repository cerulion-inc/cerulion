#!/bin/sh
# Render the Homebrew formula for one release to stdout.
#
# Usage: render_homebrew_formula.sh VERSION TAG REPOSITORY SHA256SUMS
#
# VERSION is the tag without its leading v, TAG the tag itself, REPOSITORY the
# owner/name the archives were published under, and SHA256SUMS the checksum
# file from that release. Every one of the four platform archives must appear
# in it; a missing checksum is refused rather than rendered as an empty string,
# because Homebrew would then install an unverified download.
#
# The formula reaches users only through the release workflow, which renders it
# here and opens a pull request. Rendering is a separate script so the result
# can be checked without cutting a release; tools/scripts/test_render_homebrew_formula.sh
# drives it.

set -eu

if [ "$#" -ne 4 ]; then
    printf '%s\n' 'usage: render_homebrew_formula.sh VERSION TAG REPOSITORY SHA256SUMS' >&2
    exit 2
fi

version=$1
tag=$2
repository=$3
sums=$4

if [ ! -f "$sums" ]; then
    printf 'error: no such checksum file: %s\n' "$sums" >&2
    exit 2
fi

checksum() {
    awk -v archive="$1" '$2 == archive { print $1 }' "$sums"
}

linux_x86="cerulion-${version}-x86_64-unknown-linux-gnu.tar.gz"
linux_arm="cerulion-${version}-aarch64-unknown-linux-gnu.tar.gz"
mac_x86="cerulion-${version}-x86_64-apple-darwin.tar.gz"
mac_arm="cerulion-${version}-aarch64-apple-darwin.tar.gz"

linux_x86_sha=$(checksum "$linux_x86")
linux_arm_sha=$(checksum "$linux_arm")
mac_x86_sha=$(checksum "$mac_x86")
mac_arm_sha=$(checksum "$mac_arm")

for archive in "$linux_x86" "$linux_arm" "$mac_x86" "$mac_arm"; do
    if [ -z "$(checksum "$archive")" ]; then
        printf 'error: %s has no checksum in %s\n' "$archive" "$sums" >&2
        exit 1
    fi
done

base="https://github.com/${repository}/releases/download/${tag}"

cat <<RUBY
# The release workflow writes this file.
class Cerulion < Formula
  desc "Zero-copy, deterministic communication for real-time robotics"
  homepage "https://github.com/${repository}"
  license "AGPL-3.0-only"
  version "${version}"

  on_macos do
    if Hardware::CPU.arm?
      url "${base}/${mac_arm}"
      sha256 "${mac_arm_sha}"
    else
      url "${base}/${mac_x86}"
      sha256 "${mac_x86_sha}"
    end
  end

  on_linux do
    if Hardware::CPU.arm?
      url "${base}/${linux_arm}"
      sha256 "${linux_arm_sha}"
    else
      url "${base}/${linux_x86}"
      sha256 "${linux_x86_sha}"
    end
  end

  def install
    ["cerulion", "cerulion-netd", "cerulion-connectd"].each do |binary|
      path = Dir["{,*/}#{binary}"].find { |candidate| File.file?(candidate) }
      odie "release archive is missing #{binary}" unless path
      bin.install path
    end

    # The ROS 2 Jazzy rmw and the heap hook ship in the Linux archives only
    # and sit beside the CLI, where \`cerulion ros2 run\` looks for both.
    # macOS archives carry neither: ROS 2 Jazzy has no macOS binaries.
    on_linux do
      ["librmw_cerulion.so", "libcerulion_heaphook.so"].each do |library|
        path = Dir["{,*/}#{library}"].find { |candidate| File.file?(candidate) }
        odie "release archive is missing #{library}" unless path
        bin.install path
      end
    end

    # The license notices the archive carries. Installing these binaries
    # redistributes the dependency graph they statically link, and most of
    # those licenses require the license text and the copyright notice to
    # accompany a binary distribution. They land in the formula's doc
    # directory, the Homebrew counterpart of the Debian package's
    # /usr/share/doc/cerulion. Missing ones stop the install rather than
    # producing an installation with nothing to point a reader at.
    ["LICENSE", "NOTICE", "LICENSE-BSD-3-CLAUSE",
     "THIRD-PARTY-LICENSES.md"].each do |notice|
      path = Dir["{,*/}#{notice}"].find { |candidate| File.file?(candidate) }
      odie "release archive is missing #{notice}" unless path
      doc.install path
    end

    # The install-provenance marker the CLI reports as its install method in
    # usage telemetry: share/cerulion/install.json one level above its bin.
    (share/"cerulion").mkpath
    (share/"cerulion/install.json").write "{\"method\":\"brew\",\"version\":\"#{version}\"}\n"

    # Homebrew runs the install with HOME pointing at a directory it deletes
    # afterwards, so a formula cannot put a Rust toolchain in the home folder
    # of the person installing it. Carry the archive's own setup helper and
    # the compiler fingerprint it reads, and hand over one command instead.
    helper = Dir["{,*/}install_rust.sh"].find { |candidate| File.file?(candidate) }
    metadata = Dir["{,*/}rustc-version.txt"].find { |candidate| File.file?(candidate) }
    odie "release archive carries only part of its Rust setup" if helper.nil? != metadata.nil?
    if helper
      libexec.install helper
      libexec.install metadata
      (bin/"cerulion-install-rust").write <<~WRAPPER
        #!/bin/sh
        if [ "\$#" -ne 0 ]; then
            echo "usage: cerulion-install-rust" >&2
            exit 2
        fi
        # The helper speaks for the archive installer, whose failure arms say
        # the Cerulion binaries were left alone. Here they are already
        # installed and nothing was staged to replace, so say what happened.
        /bin/sh "#{opt_libexec}/install_rust.sh" "#{opt_libexec}/rustc-version.txt"
        status=\$?
        if [ "\$status" -ne 0 ]; then
            echo "The Cerulion programs are installed; only the compiler setup failed." >&2
        fi
        exit "\$status"
      WRAPPER
      chmod 0755, bin/"cerulion-install-rust"
    end

    generate_completions_from_executable(bin/"cerulion", "completions")
  end

  # Printed unconditionally, so \`brew info cerulion\` carries the one extra
  # command before anyone installs anything.
  def caveats
    <<~TEXT
      Running Cerulion needs nothing further. Building your own nodes needs the
      exact Rust compiler that built this release, which is one command, once:

        cerulion-install-rust

      Then put Cargo's programs on your PATH, in your shell startup file:

        export PATH="\${CARGO_HOME:-\$HOME/.cargo}/bin:\$PATH"

      That is a separate step because Homebrew itself never writes into your
      home folder, and a Rust toolchain lives there.
    TEXT
  end

  test do
    assert_match version.to_s, shell_output("#{bin}/cerulion --version")
  end
end
RUBY
