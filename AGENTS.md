# FabOS working rules

These rules apply to every change unless the operator explicitly overrides one.

## Build for one change, not many copies

- Keep the codebase consistent. Prefer shared services, templates, components, styles and configuration over page-specific copies.
- When the same change belongs on several pages, create or extend the single shared source that drives all of them.
- Preserve existing architecture and conventions; do not add a parallel implementation of something the application already provides.

## Design is a system

- Keep the product visually and behaviourally consistent.
- Treat `/admin/design` and its examples as the design reference. Add reusable examples there when introducing a new recurring pattern.
- Reuse the design system and shared templates/styles instead of inventing page-local variants.

## Make the common path simple

- Minimise the clicks and choices needed to complete a task.
- Keep complexity out of the normal path; reveal advanced controls only when they are needed.

## Definition of done

- Artemis hosts the environment used for operator review. Every finished fix or feature must be deployed there and verified there; do not treat a local-only change as delivered.
- Update the relevant Markdown documentation in `FabApp/docs/` in every work session. `ROADMAP.md`, `PROJECT_STATE.md`, and `HISTORY.md` are rendered by the site, so keep the public roadmap view accurate.
- Commit every finished fix or feature.
- Follow the documented Artemis deployment procedure: deploy narrowly selected files to CT 210, never use `deploy.sh`, and clear/compile/restart only as required by the change.
