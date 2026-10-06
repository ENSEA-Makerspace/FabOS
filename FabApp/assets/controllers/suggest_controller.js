import { Controller } from '@hotwired/stimulus';

/**
 * Propose ce qui existe déjà au lieu d'une case vide (0.7.0, « Créer une machine »).
 *
 * Se pose sur le CHAMP lui-même et lui ajoute une rangée de tuiles `.ml-tile` :
 *
 *   <input data-controller="suggest"
 *          data-suggest-words-value='{"Impression 3D": 16, "Textile": 4}'
 *          data-suggest-mode-value="one">
 *
 * - `mode: one` — le champ porte UNE valeur ; une tuile la pose, un second clic
 *   l'efface. Avec `other`, le champ se cache tant qu'une tuile suffit et une tuile
 *   « Autre… » le rouvre : la saisie libre reste possible, elle n'est plus le défaut.
 * - `mode: many` — un `<textarea>` « un mot par ligne » devient des étiquettes : les
 *   mots connus se cliquent, un champ en ajoute d'autres (autocomplétés sur TOUS les
 *   mots, pas seulement les tuiles). Le `<textarea>` reste la valeur postée.
 * - `checks` + `checksName` — choisir une valeur coche les cases qu'elle amène
 *   d'habitude (une catégorie propose ses badges). Une proposition : tout se décoche.
 *
 * ⚠️ Progressif par construction : tout est bâti dans `connect()` et défait dans
 * `disconnect()`. Sans JavaScript, le champ est le champ.
 * ⚠️ Les libellés (« Autre… », « Ajouter… ») viennent du `<form>` (`data-suggest-*`),
 * traduits par le gabarit : ce fichier ne contient aucun texte.
 */
export default class extends Controller {
    static values = {
        words: Object,
        mode: { type: String, default: 'one' },
        max: { type: Number, default: 12 },
        other: Boolean,
        checks: Object,
        checksName: String,
    };

    connect() {
        this.built = [];
        this.texts = this.element.form ? this.element.form.dataset : {};
        this.row = this.make('div', 'ml-cats');
        Object.entries(this.wordsValue).slice(0, this.maxValue).forEach(([word, count]) => this.row.append(this.tile(word, count)));
        this.element.before(this.row);
        this.row.addEventListener('click', (event) => {
            const tile = event.target.closest('.ml-tile');
            if (tile) { this.clicked(tile); }
        });
        if (this.modeValue === 'many') { this.connectMany(); } else { this.connectOne(); }
        this.render();
    }

    disconnect() {
        this.built.forEach((node) => node.remove());
        this.element.hidden = false;
        if (this.ownHelp) { this.ownHelp.hidden = false; }
        this.element.removeEventListener('input', this.onInput);
    }

    // ---- une valeur -------------------------------------------------------

    connectOne() {
        if (this.otherValue) {
            this.otherTile = this.tile(this.texts.suggestOther || '…', null);
            delete this.otherTile.dataset.word;
            this.row.append(this.otherTile);
            // Une valeur hors des tuiles (catégorie archivée, erreur de saisie) : le champ reste visible.
            this.element.hidden = this.element.value === '' || this.known(this.element.value);
        }
        this.onInput = () => this.render();
        this.element.addEventListener('input', this.onInput);
    }

    // ---- plusieurs valeurs ------------------------------------------------

    connectMany() {
        this.element.hidden = true;
        const list = this.make('datalist');
        list.id = this.element.id + '-words';
        Object.keys(this.wordsValue).forEach((word) => { const option = document.createElement('option'); option.value = word; list.append(option); });
        this.adder = this.make('input');
        this.adder.type = 'text';
        this.adder.autocomplete = 'off';
        this.adder.placeholder = this.texts.suggestAdd || '';
        this.adder.setAttribute('aria-label', this.adder.placeholder);
        this.adder.setAttribute('list', list.id);
        const help = this.make('p', 'form-help');
        help.textContent = this.texts.suggestHelp || '';
        this.element.before(this.adder, help, list);
        // L'aide du champ d'origine (« un par ligne ») décrit le <textarea> qu'on vient de cacher.
        this.ownHelp = [...this.element.parentElement.querySelectorAll('.form-help')].find((node) => node !== help) || null;
        if (this.ownHelp) { this.ownHelp.hidden = true; }
        this.adder.addEventListener('keydown', (event) => {
            if (event.key === 'Enter' || event.key === ',') { event.preventDefault(); this.commit(); }
        });
        // Choisir dans l'autocomplétion pose la valeur exacte d'une option : on l'ajoute aussitôt.
        this.adder.addEventListener('input', () => { if (this.adder.value in this.wordsValue) { this.commit(); } });
        this.adder.addEventListener('blur', () => this.commit());
    }

    commit() {
        const now = this.lines();
        this.adder.value.split(',').map((word) => word.trim()).filter(Boolean).forEach((word) => { if (!now.includes(word)) { now.push(word); } });
        this.adder.value = '';
        this.write(now);
    }

    lines() {
        return this.element.value.split('\n').map((word) => word.trim()).filter(Boolean);
    }

    write(words) {
        this.element.value = words.join('\n');
        this.render();
    }

    // ---- commun -----------------------------------------------------------

    clicked(tile) {
        if (this.modeValue === 'many') {
            const now = this.lines();
            const word = tile.dataset.word;
            this.write(now.includes(word) ? now.filter((other) => other !== word) : [...now, word]);
            return;
        }
        if (tile === this.otherTile) {
            if (this.known(this.element.value)) { this.element.value = ''; }
            this.element.hidden = false;
            this.render();
            this.element.focus();
            return;
        }
        this.element.value = this.element.value === tile.dataset.word ? '' : tile.dataset.word;
        if (this.otherValue) { this.element.hidden = true; }
        if (this.element.value !== '') { this.check(this.element.value); }
        this.render();
    }

    check(word) {
        if (!this.hasChecksNameValue || !this.element.form) { return; }
        const ids = (this.checksValue[word] || []).map(String);
        this.element.form.querySelectorAll(`input[type="checkbox"][name="${this.checksNameValue}"]`).forEach((box) => { box.checked = ids.includes(box.value); });
    }

    render() {
        const tiles = [...this.row.querySelectorAll('[data-word]')];
        if (this.modeValue === 'many') {
            const now = this.lines();
            tiles.forEach((tile) => { if ('added' in tile.dataset && !now.includes(tile.dataset.word)) { tile.remove(); } });
            now.forEach((word) => {
                if (![...this.row.querySelectorAll('[data-word]')].some((tile) => tile.dataset.word === word)) {
                    const tile = this.tile(word, null);
                    tile.dataset.added = '';
                    this.row.append(tile);
                }
            });
            this.row.querySelectorAll('[data-word]').forEach((tile) => this.press(tile, now.includes(tile.dataset.word)));
            return;
        }
        tiles.forEach((tile) => this.press(tile, tile.dataset.word === this.element.value));
        if (this.otherTile) { this.press(this.otherTile, !this.element.hidden && !this.known(this.element.value)); }
    }

    known(word) {
        return Object.keys(this.wordsValue).slice(0, this.maxValue).includes(word);
    }

    press(tile, on) {
        tile.classList.toggle('is-on', on);
        tile.setAttribute('aria-pressed', on ? 'true' : 'false');
    }

    tile(word, count) {
        const tile = this.make('button', 'ml-tile', false);
        tile.type = 'button';
        tile.dataset.word = word;
        const label = document.createElement('b');
        label.className = 'ml-tile-label';
        label.textContent = word;
        tile.append(label);
        if (count !== null) {
            const small = document.createElement('b');
            small.className = 'ml-tile-count';
            small.textContent = count;
            tile.append(small);
        }
        return tile;
    }

    make(tag, className = '', track = true) {
        const node = document.createElement(tag);
        if (className) { node.className = className; }
        if (track) { this.built.push(node); }
        return node;
    }
}
