(() => {
    'use strict';

    const showcase = document.querySelector('[data-badge-showcase]');
    const overlay = document.querySelector('[data-badge-unlock-overlay]');

    if (!showcase || !overlay) {
        return;
    }

    const unlocked = showcase.dataset.badgeUnlocked === '1';
    const badgeId = showcase.dataset.badgeId || '';
    const userId = showcase.dataset.userId || '';
    const obtainedAt = showcase.dataset.badgeObtainedAt || '';

    if (!unlocked || !badgeId || !userId || !obtainedAt) {
        return;
    }

    const storageKey = `fabos-badge-unlock-seen:${userId}:${badgeId}:${obtainedAt}`;
    let alreadySeen = false;

    try {
        alreadySeen = window.localStorage.getItem(storageKey) === '1';
    } catch (_) {
        alreadySeen = false;
    }

    if (alreadySeen) {
        return;
    }

    const close = () => {
        overlay.classList.remove('is-visible');
        window.setTimeout(() => {
            overlay.hidden = true;
        }, 240);

        try {
            window.localStorage.setItem(storageKey, '1');
        } catch (_) {
            // Le stockage local peut être désactivé : l'animation reste simplement non mémorisée.
        }
    };

    window.setTimeout(() => {
        overlay.hidden = false;
        requestAnimationFrame(() => {
            overlay.classList.add('is-visible');
            showcase.classList.add('is-unlocking');
        });
    }, 350);

    overlay.querySelector('[data-badge-unlock-close]')?.addEventListener('click', close);
    overlay.addEventListener('click', (event) => {
        if (event.target === overlay) {
            close();
        }
    });

    document.addEventListener('keydown', (event) => {
        if (event.key === 'Escape' && !overlay.hidden) {
            close();
        }
    });
})();
