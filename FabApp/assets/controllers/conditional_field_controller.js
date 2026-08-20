import { Controller } from '@hotwired/stimulus';

/**
 * Show a field only when it can do something (S146, phase review).
 *
 * 🔴 **The finding this answers.** The event form asked "Nombre de séances" even
 * when "Une seule fois" was selected — a field with no effect in the default case,
 * which is precisely the "champ non indispensable" the review mandate forbids. Its
 * help text said so, but a sentence explaining why a control is inert is worse than
 * not drawing the control.
 *
 * ⚠️ **Progressive by construction**: the hiding happens in `connect()`, so without
 * JavaScript every field is visible and the form behaves exactly as it did. A field
 * hidden by CSS the server did not know about would be a field somebody could not
 * fill in and could not see was expected — the opposite of the intent.
 *
 * ⚠️ **It hides, it never clears.** Choosing a repeat, typing 4, then going back to
 * "Une seule fois" must not silently discard the 4: switching back shows it again,
 * still 4. Blanking it would be the re-entry rule broken by a different door.
 *
 * ⚠️ Generic on purpose. It knows "a source control" and "dependents that list the
 * values which reveal them", and nothing about events or repeats.
 *
 *   <form data-controller="conditional-field">
 *     <select data-conditional-field-target="source"
 *             data-action="change->conditional-field#apply">…</select>
 *     <div data-conditional-field-target="dependent"
 *          data-conditional-field-show-when="week two_weeks">…</div>
 */
export default class extends Controller {
    static targets = ['source', 'dependent'];

    connect() {
        this.apply();
    }

    disconnect() {
        // Leave the form as we found it: a re-render must not inherit our hiding.
        this.dependentTargets.forEach((field) => { field.hidden = false; });
    }

    apply() {
        const value = this.hasSourceTarget ? this.sourceTarget.value : '';

        this.dependentTargets.forEach((field) => {
            const shown = (field.dataset.conditionalFieldShowWhen || '').split(' ').filter(Boolean);
            field.hidden = shown.length > 0 && !shown.includes(value);
        });
    }
}
