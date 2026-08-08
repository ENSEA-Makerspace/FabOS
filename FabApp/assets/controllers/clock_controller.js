import { Controller } from '@hotwired/stimulus';

/*
 * L'horloge des affichages kiosk.
 *
 * Remplace trois copies écrites à la main : `kiosk-entries` et `kiosk-stats`
 * étaient identiques au caractère près, `kiosk-events` en était la variante à
 * deux éléments. Aucune des trois ne nettoyait son `setInterval`.
 *
 * ⚠️ Le fuseau vient du serveur, pas du navigateur. Un kiosk est un Raspberry Pi
 * accroché au mur, et un Pi dont le fuseau n'a jamais été configuré afficherait
 * UTC sans que personne ne s'en aperçoive. Il n'est pas non plus écrit en dur :
 * c'est le réglage de l'opérateur, rendu dans `data-clock-timezone-value`.
 *
 * Usage :
 *   <body data-controller="clock" data-clock-timezone-value="{{ lab_timezone() }}">
 *     <div data-clock-target="time">--:--</div>
 *
 * `data-clock-interval-value` pour changer la cadence (10 s par défaut).
 */
export default class extends Controller {
    static targets = ['time'];
    static values = {
        interval: { type: Number, default: 10000 },
        timezone: { type: String, default: '' },
    };

    connect() {
        this.formatter = new Intl.DateTimeFormat('fr-FR', {
            hour: '2-digit',
            minute: '2-digit',
            hour12: false,
            // Une valeur vide laisse Intl utiliser le fuseau du navigateur — le
            // repli le moins mauvais si l'attribut manque : mieux vaut l'heure de
            // la machine que l'UTC du serveur sur un mur.
            ...(this.timezoneValue ? { timeZone: this.timezoneValue } : {}),
        });

        // Immédiatement, puis à intervalle. Les versions écrites à la main
        // attendaient le premier intervalle, donc l'heure rendue par le serveur
        // restait affichée dix secondes de plus qu'il ne fallait.
        this.render();
        this.timer = window.setInterval(() => this.render(), this.intervalValue);
    }

    disconnect() {
        window.clearInterval(this.timer);
    }

    render() {
        const now = this.formatter.format(new Date());

        this.timeTargets.forEach((element) => {
            element.textContent = now;
        });
    }
}
