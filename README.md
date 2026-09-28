# WoMAPF 2027 workshop website

Website for the 8th International Workshop on Multi-Agent Path Finding,
accepted as an AAAI-27 workshop in Montréal, Canada.

Workshop acceptance is confirmed. Dates, speakers, and other details still
being finalized remain marked “To be announced” until confirmed.

## Editing the site

Most routine updates are separated from the layout:

- `_config.yml`: site title, description, public URL, and GitHub Pages base path
- `_data/dates.yml`: important dates
- `_data/topics.yml`: research-scope cards
- `_data/past_editions.yml`: links to previous workshop websites
- `_data/organization.yml`: organizers, advisors, and contact address
- `_data/speakers.yml`: invited speaker profiles, talk titles, and abstracts
- `_data/schedule.yml`: workshop session start times and activities
- `index.html`: workshop overview, scope, and important dates
- `program/index.html`: schedule, speakers, and panel discussion
- `call/index.html`: topics and submission guidance
- `organizers/index.html`: organizers, advisory committee, and contact
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

The implementation uses a small, self-contained Jekyll structure with separate
Home, Program, Call for Papers, and Contact pages. Shared navigation,
layout, and data files keep the site easy to edit as workshop details are finalized.

The information architecture was informed by the WoMAPF 2025 and 2026 sites,
with improvements for mobile navigation, semantic headings,
keyboard focus, compact content, and series history.
