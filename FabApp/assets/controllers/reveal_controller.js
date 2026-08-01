import { Controller } from '@hotwired/stimulus';

/*
 * "Voir plus" for a catalogue grid.
 *
 * Lives in the shared shell (`_catalogue.html.twig`, `reveal_after: N`) rather than
 * on any one page, so the seven catalogues cannot each grow their own toggle that
 * behaves slightly differently.
 *
 * ⚠️ It hides cards it did not create and does not know: the grid is filled by the
 * shell's `cards` block, so this controller only ever counts children and toggles a
 * class. It must never assume what a card contains.
 *
 * ⚠️ Progressive by construction: the hiding happens in `connect()`, so without JS
 * every card is visible and the button is a control that does nothing visible
 * — better than a page that silently shows half its contents. The extra cards stay
 * in the DOM either way, so in-page search, print and screen readers reach them.
 */
export default class extends Controller {
    static targets = ['button'];
    static values = { limit: { type: Number, default: 6 } };

    connect() {
        this.expanded = false;
        this.apply();
    }

    disconnect() {
        // Leave the DOM as we found it, so a re-render does not inherit our class.
        this.cards.forEach((card) => card.classList.remove('is-hidden'));
    }

    toggle() {
        this.expanded = !this.expanded;
        this.apply();
    }

    apply() {
        this.cards.forEach((card, index) => {
            card.classList.toggle('is-hidden', !this.expanded && index >= this.limitValue);
        });

        if (this.hasButtonTarget) {
            const button = this.buttonTarget;
            button.textContent = this.expanded ? button.dataset.revealLessValue : button.dataset.revealMoreValue;
            button.setAttribute('aria-expanded', String(this.expanded));
        }
    }

    get cards() {
        const grid = this.element.querySelector('.ml-grid');

        return grid ? Array.from(grid.children) : [];
    }
}
