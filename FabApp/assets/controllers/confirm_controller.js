import { Controller } from '@hotwired/stimulus';

/*
 * "Are you sure?" on a destructive form, in one place.
 *
 * Replaces `onsubmit="return confirm('Supprimer cet espace ?')"`, which was
 * pasted onto every delete form in the admin — inline JS, against the standing
 * Stimulus rule, and with the sentence embedded in an HTML attribute where no
 * translator was ever going to find it. The message is now a value, so the
 * caller passes `|trans` output and the five catalogues stay in step.
 *
 * ⚠️ It cancels the *submit event*, it does not disable the button. A disabled
 * control is the thing the house rule forbids; a form that asks first and then
 * submits is a control that works.
 *
 * ⚠️ No-JS behaviour is unchanged from what it replaced: `onsubmit` needed a
 * script engine too, so a browser without one has always submitted these forms
 * straight through. This is not a security control — the CSRF token and the
 * server-side permission check are. It is a guard against a slip of the mouse.
 */
export default class extends Controller {
    static values = { message: String };

    ask(event) {
        const message = this.messageValue;
        if (!message) {
            return;
        }

        if (!window.confirm(message)) {
            event.preventDefault();
        }
    }
}
