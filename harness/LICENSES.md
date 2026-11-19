# Third-party licences

The harness runs other people's software and data. Nothing on this page is legal advice. Each row says what the
piece is, the licence its own source states, and where that statement was read. Nothing here was written from
memory: every licence was read from the piece's own licence file, its package metadata, or its own page, on
2026-11-18, most of them inside an image built from `container/Dockerfile` on that day. The image read was
the Linux x86-64 one. A build for another platform (arm64, or the install without Docker) fetches other builds of
the same packages, whose bundled libraries can differ; those were not read.

## The harness's own code

Copyright (c) 2025-2026 Cerulion Inc. and the Cerulion Contributors

This folder is part of Cerulion. In the words of the Cerulion repository's own notice (its `docs/legal/NOTICE`,
[NOTICE](../docs/legal/NOTICE)): "This product is licensed under the GNU Affero General Public
License v3.0 only (AGPL-3.0-only). See the LICENSE file for the full text. A separate commercial license is also
available from Cerulion." The commercial license: [COMMERCIAL.md](../docs/legal/COMMERCIAL.md); commercial
questions: licensing@cerulion.com. Contributions are accepted under the repository's Contributor License
Agreement ([Individual CLA](../.github/CLA/individual.md), [Corporate CLA](../.github/CLA/corporate.md)),
which its contributing guide describes as "a license *grant*, not a copyright *assignment*".

The harness's code is under the GNU Affero General Public License, version 3 only (`AGPL-3.0-only`). The
full text is in [LICENSE](../LICENSE). Every Python and shell file of ours and `container/Dockerfile` carry the
line `SPDX-License-Identifier: AGPL-3.0-only`. One file of the measured code,
`measured/rcp/bench/ext/libero/native.py`, which the plain setup's tree runs too, also holds the prompt and tool texts of a paper under CC BY 4.0 and is marked
`AGPL-3.0-only AND CC-BY-4.0`. Files that are not code (this page, the README, the practice and brief texts, the
body records, the provenance lists, the requirement and file lists) carry no such line and are under the same
licence. No third-party file in this folder carries our licence line; there is no third-party file in it.

If you run a changed copy of this code as a network service, section 13 of that licence asks you to offer your
users the source of your version.

## How to read "fits AGPL-3.0-only"

The column says whether a piece under that licence may be used together with code under AGPL-3.0-only and passed
on with it. "yes" rests on the Free Software Foundation's licence list (gnu.org/licenses/license-list.html,
read 2026-11-18), which calls the MIT (Expat and X11), BSD 2-clause and 3-clause, Apache-2.0, Python (PSF),
Zlib, ISC, Boost, CC0, FreeType and Unicode licences compatible with version 3 of the GNU GPL, says the same of
LGPL-2.1 and LGPL-3.0, says MPL-2.0 is compatible through its section 3.3 (which names the GNU AGPL version 3),
and says CC BY 4.0 is compatible with all versions of the GNU GPL. The same list says the terms of the GNU AGPL
version 3 are those of GPL version 3 with one added paragraph. "separate program" means the piece is not combined
with our code: it runs as its own program or sits in the same image, which the licence calls an aggregate.
Anything that is not a plain "yes" is listed again under "Open points" at the end.

## The benchmark and the simulator

| piece | what it is here | licence | read from | fits AGPL-3.0-only |
|---|---|---|---|---|
| LIBERO, commit 8f1084e | the base benchmark: task definitions, start states, scene and object assets, the success check | code: MIT, Copyright (c) 2023 Lifelong Robot Learning. Datasets: CC BY 4.0 | the repository's LICENSE file and the licence table in its README, in the built image | yes for the code; the asset files are an open point (below) |
| LIBERO-PRO, commit eafdb80 | the perturbed suites (objects swapped, task changed) built on LIBERO | code: MIT (its LICENSE file is the same file as LIBERO's, Copyright (c) 2023 Lifelong Robot Learning). Datasets: CC BY 4.0 | the repository's LICENSE file and the licence table in its README, in the built image | yes for the code; the asset files are an open point (below) |
| LIBERO-PRO data files, Hugging Face dataset zhouxueyang/LIBERO-Pro, revision c86fc3b | 160 task and start-state files (80 task files, 80 start-state files) of the swap and task suites | CC BY 4.0 | the dataset card at that revision: "License: CC-BY-4.0 (for dataset artifacts), MIT (for codebase)" and the card's licence field | yes (data, kept apart from the code) |
| robosuite 1.4.0 | the simulation framework LIBERO builds on, with the robot and gripper models | MIT, Copyright (c) 2022 Stanford Vision and Learning Lab and UT Robot Perception and Learning Lab. Two model files under its assets state their own licence in the file: one BSD, one Apache 2.0 | the LICENSE file in the installed package; the two `<license>` lines in its asset files | yes |
| MuJoCo 3.2.3 (simulator) and 4.0.0 (episode server; installed as mujoco, mujoco-core and mujoco-studio) | the physics engine | Apache-2.0; the libraries it bundles are listed with their licences in its LICENSES_THIRD_PARTY.md (of those read: Abseil under Apache-2.0 and Eigen under MPL-2.0) | the LICENSE and LICENSES_THIRD_PARTY.md files in the installed packages | yes |
| The zero-shot system prompt and tool descriptions of the plain setup | served to the model unchanged by the plain setup | CC BY 4.0 | the paper's own page: May Espinola, Qifan Zhang and Yu Xiang, "From Reasoning to Action: Benchmarking Frontier AI Models on LIBERO-10 Robot Manipulation Tasks", arXiv:2610.30051v1, whose licence link is creativecommons.org/licenses/by/4.0; Figures 2 and 3 | yes (text, credited in the file that holds it) |

How each is delivered. The harness folder does not contain LIBERO, LIBERO-PRO, robosuite or MuJoCo. install.sh
fetches them from their own sources at the pinned revisions (benchmarks/fetch.sh, the requirement files). The container
image you build holds copies of all of them: if you pass that image on, their licences and notices go with it.

Attribution kept in the code. The plain setup carries the paper's prompt and tool texts with their attribution
(measured/rcp/bench/ext/libero/native.py, which the plain setup's tree runs too). CC BY 4.0 asks that the credit stay and
that changes be indicated: the texts are served verbatim; the embodiment name in the prompt is filled per cell.

## The base image and the build tools

| piece | what it is here | licence | read from | fits AGPL-3.0-only |
|---|---|---|---|---|
| Ubuntu 24.04 base image and the packages installed on it | the operating system of the image: 179 packages | each package's own; the full list is the last table on this page | /usr/share/doc/PACKAGE/copyright of every package in the built image | separate programs and system libraries; see the notes under that table |
| uv 0.13.7 | installs the two Python environments during the build; the binary stays in the image (install.sh fetches the same release into bx/uv where no uv is on PATH) | MIT OR Apache-2.0 | the `license` line of its Cargo.toml and its LICENSE-MIT and LICENSE-APACHE files at the tag 0.13.7 | separate program |
| CPython 3.10.22, installed by uv | the simulator's interpreter | Python Software Foundation licence | lib/python3.10/LICENSE.txt in the install. The install is a prebuilt build that also carries Tcl/Tk and other libraries; their licence files are not in the install and were not read | yes for CPython; the bundled libraries are an open point |
| CPython 3.12.3, from Ubuntu | the episode server's interpreter | Python Software Foundation licence, with the notices of the code it includes | the copyright file of the python3.12 package (a free-text file) | yes |
| tini 0.19.0 | the image's entry point; reaps finished processes | MIT (the file says Expat) | the copyright file of the tini package | separate program |
| git, curl, ca-certificates | fetch the benchmark during the build | git: GPL-2 with parts under other licences; curl: the curl licence with parts under BSD and ISC; ca-certificates: GPL-2+ and MPL-2.0 | their copyright files | separate programs |

The model client. The operator loop calls your model endpoint with Python's standard library; no model maker's
client library is installed or fetched.

## Libraries bundled inside the Python wheels

Some wheels carry compiled libraries of their own. These were read from the notices inside the wheels.

| wheel | bundled pieces and the licence each notice states | fits AGPL-3.0-only |
|---|---|---|
| opencv-python 4.6.0.66 (simulator) and opencv-python-headless 5.0.0.93 (episode server) | OpenCV: Apache-2.0. FFmpeg: LGPL-2.1. libvpx, libwebp, libpng, bzip2: BSD-style or their own permissive notices. FreeType: the FreeType licence. Qt 5 (in the 4.6.0.66 wheel only): LGPL-3.0. libcrypto and libssl (OpenSSL 1.1.1): the OpenSSL licence together with the SSLeay licence | yes for all but one: the OpenSSL and SSLeay pair is called incompatible with the GNU GPL by the list named above. Open point 1 |
| numpy 1.22.4 and 2.5.4, scipy 1.13.1 and 1.18.1 | OpenBLAS and LAPACK: BSD 3-clause. The GCC runtime library (libgfortran): GPL-3.0-or-later with the GCC runtime exception. libquadmath: LGPL-2.1-or-later. Small parts under MIT, Zlib and BSD | yes |
| torch 2.14.1+cpu | its metadata names Apache-2.0, Apache-2.0 WITH LLVM-exception, BSD-2-Clause, BSD-3-Clause, BSL-1.0 and MIT; it ships 86 licence files (85 of bundled libraries) and the GCC OpenMP library (libgomp) | yes by its metadata; the files were counted, not read one by one |
| h5py 3.16.0 | HDF5, the LZF filter, and compression libraries (libaec, libsz), each with its own notice in the wheel | yes (BSD-style notices in the files read) |
| pillow 12.3.0 and 13.0.0 | image and font libraries (FreeType, libpng, XZ Utils among those read); all notices are in the wheel's LICENSE file | yes |
| matplotlib 3.5.3 | fonts and colour maps with their own files (STIX, DejaVu, BaKoMa, AMS fonts, ColorBrewer, Solarized, Yorick), Qhull | yes (permissive notices; fonts and colour data, not code) |
| mujoco 3.2.3 and 4.0.0 | listed in the table above | yes |
| llvmlite 0.50.0, numba 0.68.1 | LLVM: Apache-2.0 WITH LLVM-exception; small vendored parts under MIT and BSD | yes |

## Python packages of the simulator environment (Python 3.10, 70 packages)

| package | version | licence | fits AGPL-3.0-only |
|---|---|---|---|
| absl-py | 2.5.1 | Apache-2.0 | yes |
| antlr4-python3-runtime | 4.9.3 | BSD (metadata) | yes |
| attrs | 26.1.0 | MIT | yes |
| bddl | 1.0.1 | MIT (its LICENSE file) | yes |
| cloudpickle | 2.1.0 | BSD 3-Clause License (metadata) | yes |
| cycler | 0.12.1 | BSD License (classifier) | yes |
| easydict | 1.9 | LGPL-3.0 (its LICENSE file: GNU Lesser General Public License, version 3) | yes |
| einops | 0.4.1 | MIT (its LICENSE file) | yes |
| etils | 1.13.0 | Apache Software License (classifier) | yes |
| exceptiongroup | 1.3.1 | MIT License (classifier) | yes |
| fastjsonschema | 2.22.3 | BSD-3-Clause (metadata) | yes |
| filelock | 4.1.0 | MIT | yes |
| fonttools | 4.65.0 | MIT (metadata) | yes |
| fsspec | 2026.9.0 | BSD-3-Clause | yes |
| future | 0.18.2 | MIT (metadata) | yes |
| glfw | 2.11.0 | MIT (metadata) | yes |
| gym | 0.25.2 | MIT (metadata) | yes |
| gym-notices | 0.1.0 | MIT License (classifier) | yes |
| h5py | 3.16.0 | BSD-3-Clause | yes |
| hydra-core | 1.2.0 | MIT (metadata) | yes |
| ImageIO | 2.38.0 | BSD-2-Clause | yes |
| importlib_resources | 7.1.0 | Apache-2.0 | yes |
| iniconfig | 2.3.1 | MIT | yes |
| Jinja2 | 3.1.6 | BSD License (classifier) | yes |
| jsonschema | 4.26.0 | MIT | yes |
| jsonschema-specifications | 2025.9.1 | MIT | yes |
| jupyter_core | 5.9.1 | BSD-3-Clause | yes |
| jupytext | 1.19.6 | MIT License (classifier) | yes |
| kiwisolver | 1.5.1 | BSD License (classifier) | yes |
| libero | 0.1.0 | MIT (its LICENSE file) | yes |
| llvmlite | 0.50.0 | BSD-2-Clause AND Apache-2.0 WITH LLVM-exception | yes |
| markdown-it-py | 4.3.0 | MIT License (classifier) | yes |
| MarkupSafe | 3.0.3 | BSD-3-Clause | yes |
| matplotlib | 3.5.3 | PSF (metadata) | yes |
| mdit-py-plugins | 0.7.0 | MIT License (classifier) | yes |
| mdurl | 0.1.2 | MIT License (classifier) | yes |
| mpmath | 1.3.0 | BSD (metadata) | yes |
| mujoco | 3.2.3 | Apache License 2.0 (metadata) | yes |
| nbformat | 5.11.1 | BSD License (classifier) | yes |
| networkx | 3.4.2 | BSD License (classifier) | yes |
| numba | 0.68.1 | BSD (metadata) | yes |
| numpy | 1.22.4 | BSD (metadata) | yes |
| omegaconf | 2.4.0 | BSD License (classifier) | yes |
| opencv-python | 4.6.0.66 | MIT (metadata) | yes |
| packaging | 26.3 | Apache-2.0 OR BSD-2-Clause | yes |
| pillow | 12.3.0 | MIT-CMU | yes |
| platformdirs | 4.12.4 | MIT | yes |
| pluggy | 1.7.0 | MIT (metadata) | yes |
| Pygments | 2.21.0 | BSD-2-Clause | yes |
| PyOpenGL | 3.1.10 | BSD License (classifier) | yes |
| pyparsing | 3.3.3 | MIT | yes |
| pytest | 9.2.0 | MIT | yes |
| python-dateutil | 3.0.0 | Apache-2.0 or BSD (dual, classifiers) | yes |
| PyYAML | 6.0.3 | MIT (metadata) | yes |
| referencing | 0.37.0 | MIT | yes |
| robosuite | 1.4.0 | MIT (its LICENSE file) | yes |
| rpds-py | 0.30.0 | MIT | yes |
| scipy | 1.13.1 | BSD License (classifier) | yes |
| setuptools | 78.1.0 | MIT License (classifier) | yes |
| six | 1.17.0 | MIT (metadata) | yes |
| sympy | 1.14.0 | BSD (metadata) | yes |
| termcolor | 3.3.0 | MIT | yes |
| tomli | 2.5.0 | MIT | yes |
| torch | 2.14.1+cpu | Apache-2.0 AND Apache-2.0 WITH LLVM-exception AND BSD-2-Clause AND BSD-3-Clause AND BSL-1.0 AND MIT | yes |
| tqdm | 4.70.1 | MPL-2.0 AND MIT (metadata) | yes |
| traitlets | 5.16.1 | BSD License (classifier) | yes |
| typing_extensions | 4.16.0 | PSF-2.0 | yes |
| tzdata | 2026.6 | Apache-2.0 (metadata) | yes |
| websockets | 16.1.1 | BSD-3-Clause | yes |
| zipp | 4.1.2 | MIT | yes |

LIBERO itself (the `libero` row) is installed into this environment from its pinned commit.

## Python packages of the episode server environment (Python 3.12, 38 packages, plus the harness's own code)

| package | version | licence | fits AGPL-3.0-only |
|---|---|---|---|
| annotated-types | 0.8.0 | MIT | yes |
| anyio | 4.16.0 | MIT | yes |
| attrs | 26.1.0 | MIT | yes |
| cffi | 2.1.1 | MIT-0 | yes |
| click | 8.5.1 | BSD-3-Clause | yes |
| cryptography | 50.0.2 | Apache-2.0 OR BSD-3-Clause | yes |
| glfw | 2.11.0 | MIT (metadata) | yes |
| h11 | 0.16.0 | MIT (metadata) | yes |
| httpcore2 | 2.16.0 | BSD-3-Clause | yes |
| httpx2 | 2.16.0 | BSD-3-Clause | yes |
| idna | 3.20 | BSD-3-Clause | yes |
| jsonschema | 4.26.0 | MIT | yes |
| jsonschema-specifications | 2025.9.1 | MIT | yes |
| mcp | 2.3.0 | MIT (metadata) | yes |
| mcp-types | 2.3.0 | MIT (metadata) | yes |
| mujoco | 4.0.0 | Apache-2.0 | yes |
| mujoco-core | 4.0.0 | Apache-2.0 | yes |
| mujoco-studio | 4.0.0 | Apache-2.0 | yes |
| numpy | 2.5.4 | BSD-3-Clause AND 0BSD AND MIT AND Zlib AND CC0-1.0 | yes |
| opencv-python-headless | 5.0.0.93 | Apache 2.0 (metadata) | yes |
| opentelemetry-api | 1.45.2 | Apache-2.0 | yes |
| pillow | 13.0.0 | MIT-CMU | yes |
| pycparser | 3.12 | BSD-3-Clause | yes |
| pydantic | 2.14.3 | MIT | yes |
| pydantic_core | 2.50.4 | MIT | yes |
| PyJWT | 2.15.1 | MIT | yes |
| PyOpenGL | 3.1.10 | BSD License (classifier) | yes |
| python-multipart | 0.0.33 | Apache-2.0 | yes |
| referencing | 0.37.0 | MIT | yes |
| rpds-py | 2026.9.1 | MIT | yes |
| scipy | 1.18.1 | BSD License (classifier) | yes |
| sse-starlette | 3.7.0 | BSD-3-Clause | yes |
| starlette | 1.10.0 | BSD-3-Clause | yes |
| truststore | 0.10.5 | MIT | yes |
| typing_extensions | 4.16.0 | PSF-2.0 | yes |
| typing-inspection | 0.4.5 | MIT | yes |
| uvicorn | 0.54.0 | BSD-3-Clause | yes |
| websockets | 17.4 | BSD-3-Clause | yes |

In both tables, a licence followed by "(classifier)" or "(metadata)" comes from the package's own metadata
fields, where no SPDX expression was given; one followed by "(its LICENSE file)" was read from the licence file
in the installed package; a plain expression is the package's own SPDX licence expression.

## System packages of the image (Ubuntu 24.04, 179 packages)

Read from each package's copyright file in the built image. 129 of the 179 files are in the
machine-readable format and name their licences in `License:` lines, listed here as written; 50 are free-text
files, shown with the licence words found in them. A copyright file lists every licence of every file in the
source package, tests and build scripts included, so a row often names more licences than the installed files use.

| source package | packages in the image | licences its copyright file names |
|---|---|---|
| apt | apt 2.8.4ubuntu0.1, libapt-pkg6.0t64 2.8.4ubuntu0.1 | BSD-3-clause; Expat; GPL-2; GPL-2+ |
| base-files | base-files 13ubuntu10.5 | free-text file; licence words in it: Artistic, GNU General Public License, GPL |
| base-passwd | base-passwd 3.6.3build1 | GPL-2; public-domain |
| bash | bash 5.2.21-2ubuntu4 | BSD-4-clause-UC; BSD-4-clause-UC and MIT-like; GFDL-NIV-1.3; GPL-2+; GPL-3+; GPL-3+ with Bison exception; Latex2e; MIT-like; permissive |
| util-linux | bsdutils 1:2.39.3-9ubuntu6.6, libblkid1 2.39.3-9ubuntu6.6, libmount1 2.39.3-9ubuntu6.6, libsmartcols1 2.39.3-9ubuntu6.6, libuuid1 2.39.3-9ubuntu6.6, mount 2.39.3-9ubuntu6.6, util-linux 2.39.3-9ubuntu6.6 | BSD-3-clause; BSD-4-clause; BSLA; GPL-2; GPL-2+; GPL-3+; LGPL; LGPL-2+; LGPL-2.1+; LGPL-3+; MIT; public-domain |
| ca-certificates | ca-certificates 20260816ubuntu0.24.04.2 | GPL-2+; MPL-2.0 |
| coreutils | coreutils 9.4-3ubuntu6.3 | BSD-4-clause-UC; FSFULLR; GFDL-NIV-1.3; GPL-3+; GPL-3+ and BSD-4-clause-UC; GPL-3+ and ISC; ISC |
| curl | curl 8.5.0-2ubuntu10.16, libcurl3t64-gnutls 8.5.0-2ubuntu10.16, libcurl4t64 8.5.0-2ubuntu10.16 | BSD-3-Clause; BSD-3-clause; BSD-4-Clause-UC; FSFULLR; GPL-2+ with Autoconf-data exception; GPL-2+ with Libtool exception; GPL-3+ with Autoconf-data exception; ISC; OLDAP-2.8; X11; curl |
| dash | dash 0.5.12-6ubuntu5 | BSD-3-Clause; BSD-3-clause; GPL-2+; public-domain |
| debconf | debconf 1.5.86ubuntu1 | BSD-2-clause |
| debianutils | debianutils 5.17build1 | GPL-2+; SMAIL-GPL; public-domain |
| diffutils | diffutils 1:3.10-1ubuntu0.1 | FSFAP; FSFULLR; GFDL-NIV-1.3; GPL-2+; GPL-3+; GPL-3+ and FSFULLR; GPL-3+ with autoconf exception; GPL-3+ with texinfo exception; LGPL-2.0+; LGPL-2.1+; LGPL-3.0+; LGPL-3.0+ or GPL-2+; X11; public-domain |
| dpkg | dpkg 1.22.6ubuntu6.6 | GPL-2+; public-domain-s-s-d |
| e2fsprogs | e2fsprogs 1.47.0-2.4~exp1ubuntu4.1, libcom-err2 1.47.0-2.4~exp1ubuntu4.1, libext2fs2t64 1.47.0-2.4~exp1ubuntu4.1, libss2 1.47.0-2.4~exp1ubuntu4.1, logsave 1.47.0-2.4~exp1ubuntu4.1 | Apache-2; BSD-3-Clause; GPL or MIT-US-export; GPL-2; GPL-2+ with Texinfo exception; ISC; Kazlib; LGPL-2; Latex2e |
| findutils | findutils 4.9.0-5build1 | BSD-3-clause; BSD-3-clause and/or GPL-3+; FSFAP; FSFULLR; GFDL-NIV-1.3+; GPL with automake exception; GPL-2+; GPL-2+ with Autoconf-data exception; GPL-3+; GPL-3+ with Autoconf-data exception; GPL-3+ with Bison-2.2 exception; ISC; ISC and/or LGPL-2.1+; LGPL-2+; LGPL-2.1+; LGPL-3; LGPL-3+; X11; public-domain |
| gcc-14 | gcc-14-base 14.2.0-4ubuntu2~24.04.1, libgcc-s1 14.2.0-4ubuntu2~24.04.1, libstdc++6 14.2.0-4ubuntu2~24.04.1 | free-text file; licence words in it: Artistic, BSD, GNU General Public License, GNU Library General Public License, GPL, GPL , GPL-, GPL-2, GPL-3, GPL-3.0, GPLv3, LGPL, LGPL , LGPL 2.1, LGPL-2.1, Public domain, Zlib, mit, zlib |
| git | git 1:2.43.0-1ubuntu7.4, git-man 1:2.43.0-1ubuntu7.4 | Apache-2.0; BSD-3-clause; Boost; EDL-1.0; Expat; GPL-1+ or Artistic-1; GPL-2; GPL-2+; ISC; LGPL-2+; LGPL-2.1+; Zlib; dlmalloc; mingw-runtime |
| gnupg2 | gpgv 2.4.4-2ubuntu17.6 | BSD-3-clause; CC0-1.0; Expat; GPL-2+; GPL-3+; GPL-3+ or BSD-3-clause; LGPL-2.1+; LGPL-3+; LGPL-3+ or GPL-2+; RFC-Reference; TinySCHEME; permissive |
| grep | grep 3.11-4build1 | GPL-3+ |
| gzip | gzip 1.12-1ubuntu3.2 | FSF-manpages; GFDL-1.3+-no-invariant; GPL-3+ |
| hostname | hostname 3.23+nmu2ubuntu2 | GPL-2 |
| init-system-helpers | init-system-helpers 1.66ubuntu1 | BSD-3-clause; GPL-2+ |
| acl | libacl1 2.3.2-1build1.1 | GPL-2+; LGPL-2+ |
| libassuan | libassuan0 2.5.6-1build1 | GAP; GAP~FSF; GPL-2+; GPL-2+ with libtool exception; GPL-3+; LGPL-2.1+; LGPL-3+ |
| attr | libattr1 1:2.5.2-1ubuntu0.1 | GPL-2+; LGPL-2+ |
| audit | libaudit-common 1:3.1.2-2.1ubuntu0.1, libaudit1 1:3.1.2-2.1ubuntu0.1 | GPL-2; LGPL-2.1 |
| brotli | libbrotli1 1.1.0-2ubuntu0.1 | MIT |
| libbsd | libbsd0 0.12.1-1build1.1 | BSD-2-clause; BSD-2-clause-NetBSD; BSD-2-clause-author; BSD-2-clause-verbatim; BSD-3-clause; BSD-3-clause-John-Birrell; BSD-3-clause-Regents; BSD-3-clause-Regents and BSD-2-clause-NetBSD; BSD-3-clause-author; BSD-5-clause-Peter-Wemm; Beerware; Expat; ISC; ISC-Original; public-domain |
| bzip2 | libbz2-1.0 1.0.8-5.1ubuntu0.1 | BSD-variant; GPL-2 |
| glibc | libc-bin 2.39-0ubuntu8.9, libc6 2.39-0ubuntu8.9 | free-text file; licence words in it: BSD, GNU General Public License, GNU Lesser General Public License, GPL, GPL-2, ISC, LGPL, LGPL , LGPL-2.1 |
| libcap-ng | libcap-ng0 0.8.4-2build2 | GPL-2+; GPL-3; LGPL-2.1+ |
| libcap2 | libcap2 1:2.66-5ubuntu2.4 | BSD-3-clause; BSD-3-clause or GPL-2; BSD-3-clause or GPL-2+; GPL-2; GPL-2+ |
| libxcrypt | libcrypt1 1:4.4.36-4build1 | free-text file; licence words in it: BSD, GPL, LGPL, Public domain |
| db5.3 | libdb5.3t64 5.3.28+dfsg2-7 | Artistic or BSD-3-clause; BSD-3-clause; BSD-3-clause-fjord; GPL; GPL or Artistic; GPL-3; MIT-old; Ms-PL; Sleepycat; Sleepycat and BSD-3-clause; TCL-like; X11; zlib |
| cdebconf | libdebconfclient0 0.271ubuntu3 | BSD-2-Clause; BSD-2-clause; GPL-2+ |
| libdrm | libdrm-amdgpu1 2.4.125-1ubuntu0.1~24.04.2, libdrm-common 2.4.125-1ubuntu0.1~24.04.2, libdrm-intel1 2.4.125-1ubuntu0.1~24.04.2, libdrm2 2.4.125-1ubuntu0.1~24.04.2 | free-text file with no licence name in it; not classified here |
| libedit | libedit2 3.1-20230828-1build1 | BSD-3-clause |
| mesa | libegl-mesa0 25.2.8-0ubuntu0.24.04.5, libgbm1 25.2.8-0ubuntu0.24.04.5, libgl1-mesa-dri 25.2.8-0ubuntu0.24.04.5, libglx-mesa0 25.2.8-0ubuntu0.24.04.5, mesa-libgallium 25.2.8-0ubuntu0.24.04.5 | (MIT OR Apache-2.0) AND Unicode-DFS-2016; Apache-2.0; Apache-2.0 OR MIT; BSD-2-clause; BSD-3-google; BSL; GPL; GPL-1+; GPL-2; GPL-2 or MIT; Khronos; MIT; MIT OR Apache-2.0; MLAA; SGI; Unicode-DFS-2016 |
| libglvnd | libegl1 1.7.0-1build1, libgl1 1.7.0-1build1, libglvnd0 1.7.0-1build1, libglx0 1.7.0-1build1 | Apache-2.0; BSD-1-clause; GPL; GPL-3+; MIT; public-domain |
| elfutils | libelf1t64 0.190-1.1ubuntu0.1 | BSD-2-clause; GFDL-NIV-1.3; GPL-3+; GPL-3+ with Bison exception; LGPL-2.1+; LGPL-3+ or GPL-2+ |
| liberror-perl | liberror-perl 0.17029-2 | Artistic; Artistic or GPL-1+; GPL-1+; MIT/X11 |
| expat | libexpat1 2.6.1-2ubuntu0.6 | MIT |
| libffi | libffi8 3.4.6-1build1 | Expat; GPL; GPL-2+; GPL-3+; MPL-1.1 or GPL-2+ or LGPL-2.1+; X11; public-domain |
| libgcrypt20 | libgcrypt20 1.10.3-2ubuntu0.2 | free-text file; licence words in it: BSD, GNU General Public License, GNU Lesser General Public License, GPL, GPL , GPL-2, GPL2, GPLv2, LGPL, LGPL , LGPLv2.1, Public domain, openssl, public domain |
| gdbm | libgdbm-compat4t64 1.23-5.1build1, libgdbm6t64 1.23-5.1build1 | GFDL-NIV-1.3+; GPL-2+; GPL-3+ |
| glib2.0 | libglib2.0-0t64 2.80.0-6ubuntu3.10 | AFL-2.0; AFL-2.0 or LGPL-2.1+; Apache-2.0 with LLVM exception; CC-BY-SA-3.0; CC0-1.0; CC0-1.0 or Mingw-PD; Expat; FSFULLR; GPL-2+; Iconv-PD; Janik-permissive; Janik-permissive and old-glib-tests; Kuchling-PD; LGPL-2+; LGPL-2.1+; LGPL-2.1+ and Kuchling-PD and Plumb-PD; LGPL-3+; Mingw-PD; Plumb-PD; Unicode-DFS-2016; bzip2-1.0.6; cmph; old-glib-tests |
| gmp | libgmp10 2:6.3.0+dfsg-2ubuntu6.1 | GPL-2+; GPL-2+ or LGPL-3+; GPL-3+; GPL-3+ with Bison exception; LGPL-3+ |
| gnutls28 | libgnutls30t64 3.8.3-1.1ubuntu3.6 | Apache-2.0; BSD-3-Clause; CC0 license; Expat; GPLv3+; LGPLv2.1+; LGPLv3+_or_GPLv2+; The main library is licensed under GNU Lesser |
| libgpg-error | libgpg-error0 1.47-3build2.1 | BSD-3-clause; GPL-3+; LGPL-2.1+; LGPL-2.1+ or BSD-3-clause; g10-permissive |
| krb5 | libgssapi-krb5-2 1.20.1-6ubuntu2.10, libk5crypto3 1.20.1-6ubuntu2.10, libkrb5-3 1.20.1-6ubuntu2.10, libkrb5support0 1.20.1-6ubuntu2.10 | free-text file; licence words in it: BSD, GNU General Public License, GPL, GPL , GPL-2, MIT, mit |
| nettle | libhogweed6t64 3.9.1-2.2build1.1, libnettle8t64 3.9.1-2.2build1.1 | Expat; GAP; GPL-2; GPL-2+; GPL-3+ with Autoconf exception; LGPL-2+; LGPL-3+; LGPL-3+ or GPL-2+; public-domain |
| icu | libicu74 74.2-1ubuntu3.1 | GPL-3; MIT |
| libidn2 | libidn2-0 2.3.7-2build1.1 | GPL-2+; GPL-3+; LGPL-3+; LGPL-3+ or GPL-2+; Unicode |
| keyutils | libkeyutils1 1.6.3-3build1 | GPL-2+; LGPL-2+ |
| openldap | libldap2 2.6.10+dfsg-0ubuntu0.24.04.1 | BSD-3-clause; BSD-3-clause-California; BSD-3-clause-variant; BSD-4-clause-California; Beerware; Expat; Expat-ISC; Expat-UNM; F5; FSF-unlimited; FSF-unlimited and GPL-2+ with Autoconf exception; FSF-unlimited and GPL-2+ with Libtool exception; FSF-unlimited and OpenLDAP-2.8; GPL-2+; GPL-2+ with Autoconf exception; GPL-2+ with Libtool exception; GPL-2+ with Libtool exception and GPL-3+ with Libtool exception and GPL-3+; GPL-3+; GPL-3+ with Autoconf exception; GPL-3+ with Libtool exception; JCG; MIT-XC; NeoSoft-permissive; OpenLDAP-2.8; OpenLDAP-2.8 and BSD-3-clause; OpenLDAP-2.8 and BSD-3-clause-variant; OpenLDAP-2.8 and BSD-4-clause-California; OpenLDAP-2.8 and Beerware; OpenLDAP-2.8 and Expat; OpenLDAP-2.8 and Expat-ISC; OpenLDAP-2.8 and Expat-UNM; OpenLDAP-2.8 and FSF-unlimited and GPL-2+ with Libtool exception; OpenLDAP-2.8 and JCG and UMich; OpenLDAP-2.8 and UMich; OpenLDAP-2.8 and UMich and F5; UMich; public-domain |
| llvm-toolchain-20 | libllvm20 1:20.1.2-0ubuntu1~24.04.3 | APACHE-2-LLVM-EXCEPTIONS; BSD-3-Clause; BSD-3-clause; MIT; Python; solar-public-domain |
| lz4 | liblz4-1 1.9.4-1build1.1 | BSD-2-clause; GPL-2+; GPL-2+ or BSD-2-clause |
| xz-utils | liblzma5 5.6.1+really5.4.5-1ubuntu0.4 | Autoconf; GPL-2; GPL-2+; LGPL-2.1+; PD; PD-debian; config-h; noderivs; none; permissive-fsf; permissive-nowarranty; probably-PD |
| libmd | libmd0 1.1.0-2build1.1 | BSD-2-clause; BSD-2-clause-NetBSD; BSD-3-clause; BSD-3-clause-Aaron-D-Gifford; Beerware; ISC; public-domain-md4; public-domain-md5; public-domain-sha1 |
| ncurses | libncursesw6 6.4+20240113-1ubuntu2.2, libtinfo6 6.4+20240113-1ubuntu2.2, ncurses-base 6.4+20240113-1ubuntu2.2, ncurses-bin 6.4+20240113-1ubuntu2.2 | BSD-3-clause; MIT/X11; X11 |
| nghttp2 | libnghttp2-14 1.59.0-1ubuntu0.4 | BSD-2-clause; Expat; GPL-3+ with autoconf exception; MIT; all-permissive |
| npth | libnpth0t64 1.6-3.1build1 | LGPL-2.1+ |
| mesa-compat | libosmesa6 25.1.7-1ubuntu2~24.04.2 | Apache-2.0; Apache-2.0 or MIT; BSD-2-clause; BSD-3-google; BSL; GPL; GPL-1+; GPL-2; GPL-2 or MIT; Khronos; MIT; MIT or public-domain; MLAA; SGI; public-domain |
| p11-kit | libp11-kit0 0.25.3-4ubuntu2.2 | Apache-2.0; BSD-3-clause; FSFAP; FSFULLR; GPL-2+ with Autoconf-data exception; GPL-3+ with Autoconf-data exception; ISC; LGPL-2.1+; X11; customFSFUL; customFSFULLRWD |
| pam | libpam-modules 1.5.3-5ubuntu5.7, libpam-modules-bin 1.5.3-5ubuntu5.7, libpam-runtime 1.5.3-5ubuntu5.7, libpam0g 1.5.3-5ubuntu5.7 | BSD-3-clause or GPL; BSD-tcp_wrappers; Beerware; GPL-2; GPL-2+; GPL-3; GPL-3+ with Bison exception; LGPL-2+; public-domain |
| libpciaccess | libpciaccess0 0.17-3ubuntu0.24.04.3 | free-text file; licence words in it: GPL |
| pcre2 | libpcre2-8-0 10.42-4ubuntu2.2 | BSD-2-clause; BSD-3-clause; BSD-3-clause-Cambridge with BINARY LIBRARY-LIKE PACKAGES exception; X11; public-domain |
| perl | libperl5.38t64 5.38.2-3.2ubuntu0.7, perl 5.38.2-3.2ubuntu0.7, perl-base 5.38.2-3.2ubuntu0.7, perl-modules-5.38 5.38.2-3.2ubuntu0.7 | Artistic; Artistic or GPL-1+ or Artistic-dist; Artistic-2; Artistic-dist; BSD-3-clause; BSD-3-clause-GENERIC; BSD-3-clause-with-weird-numbering; BSD-4-clause-POWERDOG; BZIP; CC0-1.0; DONT-CHANGE-THE-GPL; Expat; Expat or GPL-1+ or Artistic; FSFAP; GPL-1+; GPL-1+ or Artistic; GPL-1+ or Artistic or Artistic-dist; GPL-1+ or Artistic, and BSD-3-clause-GENERIC; GPL-1+ or Artistic, and BSD-4-clause-POWERDOG; GPL-1+ or Artistic, and Expat; GPL-1+ or Artistic, and Unicode; GPL-2+; GPL-2+ or Artistic; GPL-3+-WITH-BISON-EXCEPTION; LGPL-2.1; REGCOMP; REGCOMP, and GPL-1+ or Artistic; SDBM-PUBLIC-DOMAIN; TEXT-TABS; Unicode; ZLIB |
| procps | libproc2-0 2:4.0.4-4ubuntu3.3, procps 2:4.0.4-4ubuntu3.3 | GPL-2.0+; LGPL-2.0+; LGPL-2.1+ |
| libpsl | libpsl5t64 0.21.2-1.1build1 | Chromium; MIT; gnulib |
| python3-defaults | libpython3-stdlib 3.12.3-0ubuntu2.1, python3 3.12.3-0ubuntu2.1, python3-minimal 3.12.3-0ubuntu2.1, python3-venv 3.12.3-0ubuntu2.1 | free-text file; licence words in it: GNU General Public License, GPL, GPL-, PSF, PYTHON SOFTWARE FOUNDATION, Python Software Foundation, psf |
| python3.12 | libpython3.12-minimal 3.12.3-1ubuntu0.17, libpython3.12-stdlib 3.12.3-1ubuntu0.17, python3.12 3.12.3-1ubuntu0.17, python3.12-minimal 3.12.3-1ubuntu0.17, python3.12-venv 3.12.3-1ubuntu0.17 | * Permission to use this software in any way is granted without; By obtaining, using, and/or copying this software and/or its; Permission is hereby granted, free of charge, to any person; Permission is hereby granted, free of charge, to any person obtaining; Permission to use, copy, modify, and distribute this software and; Permission to use, copy, modify, and distribute this software and its; Permission to use, copy, modify, and distribute this software for any; Redistribution and use in source and binary forms, with or without; This software is provided 'as-is', without any express or implied; This software is provided as-is, without express or implied |
| readline | libreadline8t64 8.2-4build1, readline-common 8.2-4build1 | GFDL-NIV-1.3+; GPL-2+; GPL-3; GPL-3+; ISC-no-attribution |
| rtmpdump | librtmp1 2.4+20151223.gitfa8646d.1-2build7 | free-text file; licence words in it: GNU General Public License, GNU Lesser General Public License, GPL-2, LGPL-2.1, lgpl. |
| cyrus-sasl2 | libsasl2-2 2.1.28+dfsg1-5ubuntu3.1, libsasl2-modules-db 2.1.28+dfsg1-5ubuntu3.1 | BSD-2-clause; BSD-2-clause and MIT-CMU; BSD-2.2-clause; BSD-3-Clause-Attribution; BSD-3-Clause-Attribution and IBM-as-is; BSD-3-Clause-Attribution and MIT-Export; BSD-3-clause; BSD-3-clause-JANET; BSD-3-clause-JANET and BSD-3-Clause-Attribution; BSD-3-clause-PADL; BSD-3-clause-PADL and MIT-OpenVision; BSD-4-clause-UC; FSFULLR; FSFULLR and MIT-CMU; GPL-3; GPL-3+; IBM-as-is; MIT-CMU; MIT-Export; MIT-OpenVision; OpenLDAP; RSA-MD |
| libseccomp | libseccomp2 2.5.5-1ubuntu3.1 | LGPL-2.1 |
| libselinux | libselinux1 3.5-2ubuntu2.1 | GPL-2; public-domain |
| libsemanage | libsemanage-common 3.5-1build5, libsemanage2 3.5-1build5 | GPL-2; LGPL-2.1+ |
| lm-sensors | libsensors-config 1:3.6.0-9build1, libsensors5 1:3.6.0-9build1 | free-text file; licence words in it: GNU General Public License, GNU Lesser General Public License, GPL-2, LGPL-2.1 |
| libsepol | libsepol2 3.5-2build1 | GPL-2; GPL-2+; LGPL-2.1+; Zlib |
| sqlite3 | libsqlite3-0 3.45.1-1ubuntu2.9 | GPL-2+; public-domain |
| libssh | libssh-4 0.10.6-2ubuntu0.5 | BSD-2-clause; BSD-3-clause; LGPL-2.1; LGPL-2.1+~OpenSSL; LGPL-2.1+~OpenSSL or BSD-2-clause or BSD-3-clause; public-domain |
| openssl | libssl3t64 3.0.13-0ubuntu3.16, openssl 3.0.13-0ubuntu3.16 | Apache-2.0; Artistic; Artistic or GPL-1+; GPL-1+ |
| systemd | libsystemd0 255.4-1ubuntu8.18, libudev1 255.4-1ubuntu8.18 | CC0-1.0; Expat; GPL-2 with Linux-syscall-note exception; GPL-2+; LGPL-2.1+; public-domain |
| libtasn1-6 | libtasn1-6 4.19.0-3ubuntu0.24.04.2 | free-text file; licence words in it: GNU General Public License, GPL-3, GPLv3, LGPL, LGPL-2.1, LGPLv2.1 |
| libunistring | libunistring5 1.1-2build1.1 | FreeSoftware; GFDL-NIV-1.2+; GPL-2+; GPL-2+ with distribution exception; GPL-2+ with distribution exception, Expat and GPL-2+; GPL-3+; GPL-3+ or GFDL-NIV-1.2+; LGPL-3+; LGPL-3+ or GPL-2+; X11 |
| vulkan-loader | libvulkan1 1.3.275.0-1build1 | Apache-2.0; MIT; MIT or Apache-2.0 |
| wayland | libwayland-client0 1.22.0-2.1build1 | X11 |
| libx11 | libx11-6 2:1.8.7-1ubuntu0.1, libx11-data 2:1.8.7-1ubuntu0.1, libx11-xcb1 2:1.8.7-1ubuntu0.1 | free-text file; licence words in it: MIT, mit |
| libxau | libxau6 1:1.0.9-1build6 | free-text file with no licence name in it; not classified here |
| libxcb | libxcb-dri3-0 1.15-1ubuntu2, libxcb-glx0 1.15-1ubuntu2, libxcb-present0 1.15-1ubuntu2, libxcb-randr0 1.15-1ubuntu2, libxcb-shm0 1.15-1ubuntu2, libxcb-sync1 1.15-1ubuntu2, libxcb-xfixes0 1.15-1ubuntu2, libxcb1 1.15-1ubuntu2 | free-text file with no licence name in it; not classified here |
| libxdmcp | libxdmcp6 1:1.1.3-0ubuntu6 | free-text file; licence words in it: MIT |
| libxext | libxext6 2:1.3.4-1build2 | free-text file with no licence name in it; not classified here |
| libxml2 | libxml2 2.9.14+dfsg-1.3ubuntu3.10 | ISC; MIT-1 |
| libxshmfence | libxshmfence1 1.3-1build5 | free-text file with no licence name in it; not classified here |
| libxxf86vm | libxxf86vm1 1:1.1.4-1build4 | free-text file with no licence name in it; not classified here |
| xxhash | libxxhash0 0.8.2-2build1 | BSD-2-clause; BSD-2-clause or GPL-2+; GPL-2+ |
| libzstd | libzstd1 1.5.5+dfsg2-2build1.1 | BSD-3-clause; BSD-3-clause or GPL-2; Expat; GPL-2; zlib |
| shadow | login 1:4.13+dfsg1-4ubuntu3.2, passwd 1:4.13+dfsg1-4ubuntu3.2 | BSD-3-clause; GPL-1; GPL-2+; public-domain |
| mawk | mawk 1.3.4.20240123-1build1 | CC-BY-3.0; GPL-2.0-only; X11 |
| media-types | media-types 10.1.0 | ad-hoc |
| netbase | netbase 6.4 | GPL-2 |
| python-pip | python3-pip-whl 24.0+dfsg-1ubuntu1.3 | Apache-2.0; Apache-2.0 OR BSD-2; BSD-2; BSD-3; Expat; ISC; LGPL-2.1+; MPL-2; Python |
| setuptools | python3-setuptools-whl 68.1.2-2ubuntu1.2 | Apache-2.0; BSD-3-clause |
| sed | sed 4.9-2ubuntu0.24.04.1 | BSD-4-clause-UC; BSL-1; GFDL-NIV-1.3+; GPL-3+; GPL-3+ and ISC; ISC; X11; pcre |
| sensible-utils | sensible-utils 0.0.22 | All-permissive; BSD-2-clause; GPL-2+; configure; installsh |
| sysvinit | sysvinit-utils 3.08-6ubuntu3 | GPL-2.0; GPL-2.0+; GPL-3.0; LGPL-2.1+ |
| tar | tar 1.35+dfsg-3ubuntu0.4 | GPL-2+; GPL-3+; GPL-3+ with Bison exception; LGPL-2.1+; LGPL-3+ |
| tini | tini 0.19.0-1 | Expat |
| tzdata | tzdata 2026e-0ubuntu0.24.04.1 | ICU; public-domain |
| ubuntu-keyring | ubuntu-keyring 2023.11.28.1 | free-text file; licence words in it: GNU General Public License, GPL |
| unminimize | unminimize 0.2.1 | GPL-2.0+ |
| zlib | zlib1g 1:1.3.dfsg-3.1ubuntu2.3 | Zlib |

Notes on this table.

- These are the base system and the libraries the simulator's CPU renderer needs. They are separate programs or
  system libraries. Our code is not combined with any of the programs.
- Packages whose copyright file names GPL version 2 without "or later" for at least some files:
  apt, base-passwd, bsdutils, e2fsprogs, git, git-man, hostname, libapt-pkg6.0t64, libaudit-common, libaudit1, libblkid1, libbz2-1.0, libcap2, libcom-err2, libegl-mesa0, libext2fs2t64, libgbm1, libgl1-mesa-dri, libglx-mesa0, libhogweed6t64, liblzma5, libmount1, libnettle8t64, libosmesa6, libpam-modules, libpam-modules-bin, libpam-runtime, libpam0g, libselinux1, libsemanage-common, libsemanage2, libsepol2, libsmartcols1, libss2, libuuid1, libzstd1, logsave, mawk, mesa-libgallium, mount, netbase, sysvinit-utils, util-linux. Some of these are programs run as their own processes (git, mawk, hostname, mount, apt
  and others). For the libraries in the list the same file also names LGPL or permissive licences. Which files
  fall under GPL-2 was read for Mesa only (interface headers of the Linux kernel) and not for the others.
- 16 packages have a free-text copyright file that gives permission notices without a licence name:
  libdrm-amdgpu1, libdrm-common, libdrm-intel1, libdrm2, libxau6, libxcb-dri3-0, libxcb-glx0, libxcb-present0, libxcb-randr0, libxcb-shm0, libxcb-sync1, libxcb-xfixes0, libxcb1, libxext6, libxshmfence1, libxxf86vm1. They were not classified here.
- If you pass a built image on, you pass on all of these packages. Ubuntu publishes the source of each.

## The sim server

The sim server (`playground/`) is code of ours under the same licence. It adds no third-party software: it uses
the MCP SDK (`mcp`, MIT), Starlette and Uvicorn (BSD-3-Clause), which are rows of the episode server table. The
pictures it shows are rendered by MuJoCo from the LIBERO and LIBERO-PRO scenes.

## Open points

1. OpenSSL 1.1.1 inside the two OpenCV wheels. The wheels bundle libcrypto and libssl under the older OpenSSL
   and SSLeay licence pair, which the list named above calls incompatible with the GNU GPL. The libraries come
   with FFmpeg's network support inside OpenCV; the harness's code uses OpenCV for image operations and never
   opens a network stream through it. Nothing in this folder contains those libraries: install.sh downloads the
   wheels from the Python package index. The question arises for whoever passes on a built image.
2. The asset files of LIBERO and LIBERO-PRO (585 and 864 files of meshes, textures and scenes). The two
   repositories state MIT for the codebase and CC BY 4.0 for datasets and name no other licence. The asset
   folders are named `stable_hope_objects`, `stable_scanned_objects`, `turbosquid_objects`, `articulated_objects`,
   `scenes` and `textures`; no licence or credit file sits in or beside them, so the origin and terms of each
   object model were not established. This folder contains none of them: install.sh fetches them from the
   LIBERO repositories.
3. The prebuilt CPython 3.10 that uv installs carries libraries (Tcl/Tk among them) whose licence files are not
   in the install. Not read.
4. 16 system packages with a copyright file that names no licence (listed above). Not classified.
   For the system libraries whose copyright file names GPL version 2 for some files (listed above), which files
   those are was read for Mesa only.
5. torch's 85 bundled licence files were counted and not read one by one; its own metadata names only
   permissive licences.
6. easydict 1.9 is LGPL-3.0. It is used as an unmodified library by LIBERO. Passing on an image that holds it
   means passing on its licence text and letting the receiver replace the library, which a Python environment
   allows. tqdm is MPL-2.0 and MIT, used unmodified.
