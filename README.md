# WoMAPF 2027 workshop website

Initial website for the proposed 8th International Workshop on Multi-Agent
Path Finding, submitted for consideration at AAAI-27.

The public page intentionally distinguishes confirmed conference facts from
workshop details that depend on the proposal decision. Do not replace a “To be
announced” value with a date, speaker, policy, or venue until it is confirmed.

## Editing the site

Most routine updates are separated from the layout:

- `_config.yml`: site title, description, public URL, and GitHub Pages base path
- `_data/dates.yml`: proposal status and important dates
- `_data/topics.yml`: research-scope cards
- `_data/past_editions.yml`: links to previous workshop websites
- `_data/organization.yml`: organizers, advisors, and contact address
- `index.html`: section copy and tentative program description
- `assets/css/main.scss`: visual design and responsive behavior

To add an organizer after confirmation:

```yaml
organizers:
  - name: Ada Researcher
    affiliation: Example University
    url: https://example.edu/
```

## Local preview

Install the GitHub Pages bundle, then serve the site without the repository
subpath:

```bash
bundle install
bundle exec jekyll serve --baseurl ""
```

Open `http://127.0.0.1:4000/`.

## Publishing with GitHub Pages

The configured production address is `https://wo-mapf.github.io/aaai27/`.
In the repository settings, enable **Pages**, choose **Deploy from a branch**,
and publish the repository root from `main`. GitHub Pages will run Jekyll and
serve the generated site at the configured project path.

## Design decision

The implementation uses a small, self-contained Jekyll structure instead of a
large conference theme. We evaluated the actively maintained
`jekyll-theme-conference`, but its schedule, speaker, room, streaming, and PWA
features are unnecessary while the proposal is pending. The current structure
keeps the initial page easy to edit and can be expanded into dedicated program,
speaker, and paper pages after acceptance.

The information architecture was informed by the WoMAPF 2025 and 2026 sites,
with improvements for proposal status, mobile navigation, semantic headings,
keyboard focus, compact content, and series history.

