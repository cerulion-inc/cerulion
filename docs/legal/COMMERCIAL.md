# Commercial Licensing

Cerulion is **dual-licensed**. You can use it for free under the open-source
license, or buy a commercial license if the open-source terms don't fit your
business. Same code either way: the license changes your obligations, not the
software.

## Two ways to use Cerulion

| | Open source (AGPL-3.0-only) | Commercial license |
|---|---|---|
| **Cost** | Free | Paid: [contact us](#contact) for terms |
| **License** | [AGPL-3.0-only](../../LICENSE) | Separate commercial agreement |
| **Best for** | Internal use, research, evaluation, open-source projects | Shipping a closed-source product, or offering Cerulion functionality as a network service without publishing your source |
| **Source-availability obligation** | Yes: if you convey the software, or let users interact with it **over a network**, you must offer them the corresponding source (AGPL §13) | No AGPL source-availability obligation |
| **Copyleft reaches your code** | Yes: derivative works must also be AGPL | No |
| **Support** | Community: questions in [GitHub Discussions](https://github.com/cerulion-inc/cerulion/discussions), bugs in issues | Paid support & SLA available: see [Paid services](#paid-services) |

If you can comply with the AGPL, you never need to talk to us. If you can't,
that's what the commercial license is for.

## What's free (start here)

Most people don't need a commercial license. The following are all **free**
under AGPL-3.0-only, with **no obligation to release your source**:

**Q: Can I use Cerulion internally to run my robots without paying or publishing anything?**
Yes. Running Cerulion inside your own organization (on your robots, your test
rigs, your lab) is use, not distribution. The AGPL's source-availability
obligation triggers when you **convey** the software to others or expose its
functionality to third parties **over a network**. Purely internal robot
operation does neither.

**Q: Can I use it for research or in an academic project?**
Yes, free under AGPL-3.0-only. If you publish or distribute a derivative work,
the AGPL terms apply to that work.

**Q: Can I run unmodified Cerulion?**
Yes. Using the software as-is, unmodified, is free. This includes running
**unmodified MoveIt 2 on Cerulion's deterministic zero-copy transport** via
[`rmw_cerulion`](../../crates/rmw_cerulion), without changing your MoveIt 2 or ROS 2
application code.

**Q: Can I evaluate it before deciding on a license?**
Yes. Evaluate as long as you need under AGPL-3.0-only. Talk to us when (and if)
your deployment can't meet the AGPL terms.

**Q: Can I contribute?**
Yes, see [CONTRIBUTING.md](../../.github/CONTRIBUTING.md). Contributions are accepted under
our Contributor License Agreement, which supports the dual-license model. It is
a license *grant*, not a copyright *assignment*: **you keep your copyright**;
your grant lets Cerulion offer its code under both AGPL-3.0 and commercial
licenses. Individuals sign via the CLA bot on their first pull request
([full text](../../.github/CLA/individual.md)); companies execute the
[Corporate CLA](../../.github/CLA/corporate.md) via
[licensing@cerulion.com](mailto:licensing@cerulion.com).

## When you likely need a commercial license

- You want to ship a **closed-source product** that includes or links Cerulion.
- You expose Cerulion functionality **as a network service** to third parties
  and cannot offer them the corresponding source (AGPL §13).
- Your legal or customer requirements prohibit copyleft dependencies.

Not sure which side of the line you're on? [Ask us](#contact). We'd rather
answer a question than have you guess.

## Paid services

Available with a commercial license:

| Service | What you get |
|---|---|
| **Commercial license** | A non-AGPL license to use, embed, and distribute Cerulion without the AGPL source-availability and copyleft obligations. |
| **Support & SLA** | Priority support directly from the team that builds Cerulion. [Contact us](#contact) and we'll scope the right support level and response terms together. |
| **Integration & onboarding** | Hands-on help bringing Cerulion into your stack: graph design, node authoring, migrating an existing ROS 2 / MoveIt 2 application onto the zero-copy transport. |
| **Prioritized fixes** | Your bug reports and blocking issues moved to the front of the queue. |

## Contact

For commercial licensing, pricing, and support terms:

**[licensing@cerulion.com](mailto:licensing@cerulion.com)**

Please include a short description of your use case (deployment shape, whether
you distribute or offer a network service, and any support needs) so we can
point you to the right option.

> Contract terms (warranty, indemnity, governing law, and jurisdiction) are
> handled in the commercial agreement itself, not in this document.
> [Contact us](#contact) for the full terms.
