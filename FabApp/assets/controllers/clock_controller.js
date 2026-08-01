import { Controller } from '@hotwired/stimulus';

/*
 * L'horloge des affichages kiosk.
 *
 * Remplace trois copies écrites à la main : `kiosk-entries` et `kiosk-stats`
 * étaient identiques au caractère près, `kiosk-events` en était la variante à
 * deux éléments. Aucune des trois ne nettoyait son `setInterval`.
 *
 * ⚠️ Le fuseau est épinglé sur Europe/Paris plutôt que laissé au fuseau local
 * du navigateur. C'est un changement de comportement délibéré : un kiosk est un
 * Raspberry Pi accroché au mur du labo, et un Pi dont le fuseau n'a jamais été
 * configuré affiche UTC sans que personne ne s'en aperçoive. La règle du projet
 * — épingler le fuseau sur la lecture comme sur l'écriture — vaut aussi ici.
 *
 * Usage :
 *   <body data-controller="clock">
 *     <div data-clock-target="time">--:--</div>
 *
 * `data-clock-interval-value` pour changer la cadence (10 s par défaut).
 */
export default class extends Controller {
    static targets = ['time'];
    static values = {
        interval: { type: Number, default: 10000 },
    };

    connect() {
        this.formatter = new Intl.DateTimeFormat('fr-FR', {
            hour: '2-digit',
            minute: '2-digit',
            hour12: false,
            timeZone: 'Europe/Paris',
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
