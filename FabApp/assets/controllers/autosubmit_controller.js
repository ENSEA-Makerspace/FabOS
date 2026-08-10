import { Controller } from '@hotwired/stimulus';

// Submits the owning form when a finite-set control changes.
//
// ⚠️ This replaces an inline `onchange="this.form.submit()"` in
// `_venue_context.html.twig`. Inline handlers are ruled out here: new interaction
// goes in a controller, and the CSP that forbids third-party hosts is the same
// reason not to accumulate inline script.
//
// The roadmap's site-wide filter rule is the behaviour being encoded: a control
// with a finite set of values (a select, a state, a sub-venue, a preset date)
// applies immediately and keeps the URL; only free-text search needs an explicit
// submit. Without JavaScript the `<noscript>` button still submits the form, so
// the filter degrades to one extra click rather than breaking.
export default class extends Controller {
    submit() {
        this.element.closest('form')?.requestSubmit();
    }
}
