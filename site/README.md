# MANTA project site

Astro + Tailwind static site that pulls together MANTA's analysis,
renderings, and program status. Builds to `dist/` as plain static
HTML/CSS.

**Live:** <https://manta-ten.vercel.app>

**Deploy:** auto-deploys from `main` via GitHub Actions
([`.github/workflows/deploy.yml`](../.github/workflows/deploy.yml)) using
the Vercel CLI + a `VERCEL_TOKEN` GitHub secret. The workflow only
fires when `site/`, the analysis-output PNGs, or the workflow itself
changes — `analysis/` and `cad/` Python edits don't redeploy unless
they touch the embedded artifacts.

## Stack

- **Astro 6** — static-first, markdown-friendly, minimal JS shipped
- **Tailwind CSS 4** — utility-first styling
- **Bun** as the package manager (npm / pnpm work too)

## Develop

```sh
bun install
bun run dev          # http://localhost:4321
```

## Build

```sh
bun run build        # outputs to ./dist
bun run preview      # serve dist locally
```

## Sync assets from analysis pipeline

The site references PNG plots that live in the analysis tree. Sync
them into `site/public/img/` whenever those upstream artifacts change:

```sh
./scripts/sync-assets.sh
```

(See script for the file mapping.) The site is deliberately *not*
hot-coupled to the analysis Python pipeline — assets are synced
explicitly so that a site rebuild doesn't depend on the venv being
installed.

## Deploy

Any static host. The site is pure HTML/CSS/JS in `dist/` — no server
required. Examples:

- **Cloudflare Pages**: connect the repo, build command `cd site && bun install && bun run build`, output `site/dist`.
- **Vercel**: similar; the Astro adapter is auto-detected.
- **GitHub Pages**: `dist/` can be pushed to a `gh-pages` branch.

## Structure

```
site/
├── public/img/         PNG plots synced from ../analysis/.../out/ and ../cad/.../out/
├── src/
│   ├── layouts/Base.astro     base HTML shell
│   ├── components/            Section, MetricCard, StatusTable
│   ├── pages/index.astro      single-page landing site
│   └── styles/global.css      Tailwind import
├── astro.config.mjs
└── package.json
```
