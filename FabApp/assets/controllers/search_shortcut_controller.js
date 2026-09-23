import { Controller } from '@hotwired/stimulus';

/*
 * S194 — « / » place le curseur dans la recherche de l'en-tête, comme sur la
 * plupart des sites qu'un membre connaît déjà.
 * ⚠️ Jamais pendant une saisie : dans un champ, un « / » est un caractère.
 */
export default class extends Controller {
    static targets = ['input'];

    connect() {
        this.onKey = (event) => {
            if (event.key !== '/' || event.ctrlKey || event.metaKey || event.altKey) {
                return;
            }
            const el = event.target;
            if (el && (el.isContentEditable || ['INPUT', 'TEXTAREA', 'SELECT'].includes(el.tagName))) {
                return;
            }
            event.preventDefault();
            this.inputTarget.focus();
            this.inputTarget.select();
        };
        document.addEventListener('keydown', this.onKey);
    }

    disconnect() {
        document.removeEventListener('keydown', this.onKey);
    }
}
