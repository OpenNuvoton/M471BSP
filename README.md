# M471BSP SBOM Artifacts

The SBOM files in this Git repository are current distribution copies of the
formal SVN release:

- Release: `V3.03.000-21-ga1b76a29`
- Source commit: `a1b76a29ddd3b05862c9318e3b8bf0dfef9f0632`
- Formal SVN path: `bsp/m471/V3.03.000-21-ga1b76a29`
- Formal SVN revision: `43`
- Format: CycloneDX JSON 1.6

`M471BSP_Product_SBOM_cdx.json` is the Product view.
`M471BSP_TestSample_SBOM_cdx.json` is the separate sample, test, host-tool, and
vendored third-party view. The manifest records their hashes, component
summaries, generation tools, source identity, and formal SVN release.

This synchronization is artifact-only. It does not represent a new SBOM scan
source and must not trigger regeneration of the formal SVN release.
