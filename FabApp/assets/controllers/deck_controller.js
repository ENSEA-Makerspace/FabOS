import { Controller } from '@hotwired/stimulus';

/*
   Le carrousel des événements de l'accueil.

   ⚠️ **La liste défile déjà sans lui.** Le rail est un `overflow-x: auto` avec
   `scroll-snap`, donc sans JavaScript on garde un défilement horizontal natif,
   au doigt comme à la molette — ce contrôleur n'ajoute que l'avance
   automatique et les pastilles. C'est délibéré : un carrousel qui ne montre
   rien quand le script tombe est un carrousel qui cache du contenu.

   ⚠️ Il s'arrête au survol, au focus clavier et quand l'onglet passe en
   arrière-plan. Un mouvement qui continue pendant qu'on lit la carte est un
   mouvement qui vous la retire des yeux.

   ⚠️ `prefers-reduced-motion` coupe l'avance automatique entièrement, pas
   seulement l'animation du défilement : le problème n'est pas que ça glisse,
   c'est que ça bouge tout seul.
*/
export default class extends Controller {
    static targets = ['track', 'slide', 'dot'];
    static values = { interval: { type: Number, default: 6000 } };

    connect() {
        this.index = 0;
        this.reduced = window.matchMedia('(prefers-reduced-motion: reduce)').matches;

        this.onScroll = () => this.syncFromScroll();
        this.trackTarget.addEventListener('scroll', this.onScroll, { passive: true });

        this.onVisibility = () => (document.hidden ? this.pause() : this.play());
        document.addEventListener('visibilitychange', this.onVisibility);

        this.element.addEventListener('pointerenter', this.pause.bind(this));
        this.element.addEventListener('pointerleave', this.play.bind(this));
        this.element.addEventListener('focusin', this.pause.bind(this));
        this.element.addEventListener('focusout', this.play.bind(this));

        this.syncDots();
        this.play();
    }

    disconnect() {
        this.pause();
        this.trackTarget.removeEventListener('scroll', this.onScroll);
        document.removeEventListener('visibilitychange', this.onVisibility);
    }

    play() {
        // Une seule carte n'est pas un carrousel : rien à faire avancer, et une
        // pastille unique serait un bouton qui ne fait rien.
        if (this.reduced || this.slideTargets.length < 2 || this.timer) return;
        this.timer = window.setInterval(() => this.go(this.index + 1), this.intervalValue);
    }

    pause() {
        if (!this.timer) return;
        window.clearInterval(this.timer);
        this.timer = null;
    }

    /* Appelé par les pastilles. */
    select(event) {
        const to = Number(event.currentTarget.dataset.deckIndex || 0);
        this.go(to, true);
    }

    go(to, manual = false) {
        const count = this.slideTargets.length;
        if (!count) return;

        this.index = ((to % count) + count) % count;
        const slide = this.slideTargets[this.index];
        this.trackTarget.scrollTo({
            left: slide.offsetLeft - this.trackTarget.offsetLeft,
            behavior: this.reduced ? 'auto' : 'smooth',
        });
        this.syncDots();

        // Un clic est une prise en main : on rend la main à la personne plutôt
        // que de reprendre l'avance deux secondes plus tard sous ses doigts.
        if (manual) this.pause();
    }

    /* Le rail reste la source de vérité — on peut le faire défiler au doigt
       sans passer par `go()`, et les pastilles doivent suivre. */
    syncFromScroll() {
        const left = this.trackTarget.scrollLeft + this.trackTarget.offsetLeft;
        let best = 0;
        let bestGap = Infinity;
        this.slideTargets.forEach((slide, i) => {
            const gap = Math.abs(slide.offsetLeft - left);
            if (gap < bestGap) {
                bestGap = gap;
                best = i;
            }
        });
        if (best !== this.index) {
            this.index = best;
            this.syncDots();
        }
    }

    syncDots() {
        this.dotTargets.forEach((dot, i) => {
            const on = i === this.index;
            dot.classList.toggle('is-on', on);
            dot.setAttribute('aria-current', on ? 'true' : 'false');
        });
    }
}
