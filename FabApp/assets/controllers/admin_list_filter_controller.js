import { Controller } from '@hotwired/stimulus';

// Shared progressive enhancement for compact admin lists. Server-rendered rows
// remain usable without JavaScript; with it, search and category chips respond
// immediately and do not send an operator through an unnecessary page reload.
export default class extends Controller {
    static targets = ['search'];

    connect() {
        this.currentCategory = 'all';
        this.rows.forEach((row) => this.makeRowNavigable(row));
        this.apply();
    }

    filter() {
        this.apply();
    }

    search(event) {
        event.preventDefault();
        this.apply();
    }

    category(event) {
        event.preventDefault();
        this.currentCategory = event.currentTarget.dataset.adminListFilterCategory || 'all';
        this.apply();
    }

    apply() {
        const query = this.hasSearchTarget ? this.searchTarget.value.trim().toLocaleLowerCase() : '';
        this.rows.forEach((row) => {
            const matchesQuery = !query || row.textContent.toLocaleLowerCase().includes(query);
            const matchesCategory = this.currentCategory === 'all'
                || (row.dataset.adminListFilterCategory || '').split(' ').includes(this.currentCategory);
            row.hidden = !(matchesQuery && matchesCategory);
        });

        this.element.querySelectorAll('[data-admin-list-filter-tile]').forEach((tile) => {
            tile.classList.toggle('is-on', (tile.dataset.adminListFilterCategory || 'all') === this.currentCategory);
        });
    }

    get rows() {
        return Array.from(this.element.querySelectorAll('[data-admin-list-filter-row]'));
    }

    makeRowNavigable(row) {
        const href = row.dataset.adminListFilterRowHref;
        if (!href) {
            return;
        }

        row.tabIndex = 0;
        row.setAttribute('role', 'link');
        row.addEventListener('click', (event) => {
            if (event.target.closest('a, button, input, select, textarea, label, form')) {
                return;
            }
            window.location.assign(href);
        });
        row.addEventListener('keydown', (event) => {
            if (event.key === 'Enter' || event.key === ' ') {
                event.preventDefault();
                window.location.assign(href);
            }
        });
    }
}
