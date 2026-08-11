/**
 * La langue du lecteur, en un seul endroit.
 *
 * ⚠️ Les calendriers passaient la locale « fr-FR » en dur à `toLocaleDateString` — dans
 * une douzaine de fonctions. Le site sert cinq langues et le sélecteur de langue
 * écrit `lang` sur `<html>` : une constante en dur affichait donc « sam. 2 août »
 * à un lecteur allemand. Le pendant Twig de ceci est le filtre `|loc_date()`,
 * qui existe pour la raison symétrique (PHP `date()` ne parle qu'anglais).
 *
 * Le repli `fr-FR` couvre le cas où `lang` manque ; ce n'était pas possible
 * jusqu'ici puisque `base_public.html.twig` l'émet toujours.
 */
window.FABOS_LOCALE = document.documentElement.lang || 'fr-FR';

/**
 * Gestion globale du thème FabOS.
 * La préférence enregistrée dans UTILISATEUR.theme est exposée par Twig,
 * puis appliquée à l'ensemble du site via data-theme sur <html>.
 */
window.FabosTheme = (() => {
    const STORAGE_KEY = 'fabos-theme-preference';
    const allowedPreferences = new Set(['light', 'dark', 'system']);
    const mediaQuery = window.matchMedia('(prefers-color-scheme: dark)');
    let currentPreference = 'light';

    const normalise = preference => allowedPreferences.has(preference) ? preference : 'light';
    const resolve = preference => preference === 'system'
        ? (mediaQuery.matches ? 'dark' : 'light')
        : preference;

    const apply = (preference, options = {}) => {
        const normalised = normalise(preference);
        const resolved = resolve(normalised);
        const shouldPersist = options.persist !== false;

        currentPreference = normalised;
        document.documentElement.dataset.themePreference = normalised;
        document.documentElement.dataset.theme = resolved;

        if (shouldPersist) {
            try {
                localStorage.setItem(STORAGE_KEY, normalised);
            } catch (error) {
                // Le thème reste fonctionnel même si le stockage navigateur est désactivé.
            }
        }

        document.dispatchEvent(new CustomEvent('fabos:themechange', {
            detail: { preference: normalised, theme: resolved }
        }));

        return { preference: normalised, theme: resolved };
    };

    const getStoredPreference = () => {
        try {
            return normalise(localStorage.getItem(STORAGE_KEY) || 'light');
        } catch (error) {
            return 'light';
        }
    };

    const getPreference = () => currentPreference;
    const getTheme = () => resolve(currentPreference);

    const handleSystemChange = () => {
        if (currentPreference === 'system') {
            apply('system', { persist: false });
        }
    };

    if (typeof mediaQuery.addEventListener === 'function') {
        mediaQuery.addEventListener('change', handleSystemChange);
    } else if (typeof mediaQuery.addListener === 'function') {
        mediaQuery.addListener(handleSystemChange);
    }

    apply(getStoredPreference(), { persist: false });

    return { apply, getPreference, getTheme };
})();

/**
 * FabOS - JavaScript Principal
 * Version: 1.0 - Mis à jour pour la nouvelle structure
 * Date: 24 juin 2026
 * Description: Scripts pour les interactions de base (sans MQTT pour l'instant)
 */

// Attendre que le DOM soit complètement chargé
document.addEventListener('DOMContentLoaded', function() {
    // Initialiser tous les composants
    initMobileNavToggle();
    initViewToggle();
    initFavoriteToggle();
    initScrollAnimations();
    // initHeaderSearch() supprimé : la recherche du header est un <form> natif.
    initSearchFunctionality();
    initPasswordToggles();
    initFilterFunctionality();
    initThemePreference();
    initProfileThemeSwitch();
});

/**
 * Toggle du menu mobile
 */
function initMobileNavToggle() {
    const mobileNavToggle = document.querySelector('.mobile-nav-toggle');
    const mainNavbar = document.querySelector('.main-navbar');
    
    if (mobileNavToggle && mainNavbar) {
        // The button toggled a class and said nothing (S52). A screen reader was
        // told "Menu, button" whether the menu was open or shut, so the one piece
        // of state that matters was the only thing not announced.
        if (!mainNavbar.id) {
            mainNavbar.id = 'main-navbar';
        }
        mobileNavToggle.setAttribute('aria-controls', mainNavbar.id);
        mobileNavToggle.setAttribute('aria-expanded', 'false');

        const setNavOpen = (open) => {
            mainNavbar.classList.toggle('active', open);
            mobileNavToggle.classList.toggle('active', open);
            mobileNavToggle.setAttribute('aria-expanded', open ? 'true' : 'false');
        };

        mobileNavToggle.addEventListener('click', function() {
            setNavOpen(!mainNavbar.classList.contains('active'));
        });

        // Fermer le menu quand on clique en dehors
        document.addEventListener('click', function(e) {
            if (!mobileNavToggle.contains(e.target) && !mainNavbar.contains(e.target)) {
                setNavOpen(false);
            }
        });

        // Escape closes it and hands focus back to the control that opened it —
        // otherwise keyboard focus is stranded inside a menu that has gone.
        document.addEventListener('keydown', function(e) {
            if (e.key === 'Escape' && mainNavbar.classList.contains('active')) {
                setNavOpen(false);
                mobileNavToggle.focus();
            }
        });
    }
}

/**
 * Toggle entre vue grille et vue liste
 */
function initViewToggle() {
    const viewToggleBtns = document.querySelectorAll('.view-toggle .toggle-btn');
    const machinesContainer = document.getElementById('machines-container');
    
    if (viewToggleBtns.length > 0 && machinesContainer) {
        viewToggleBtns.forEach(btn => {
            btn.addEventListener('click', function() {
                // Retirer la classe active de tous les boutons
                viewToggleBtns.forEach(b => b.classList.remove('active'));
                
                // Ajouter la classe active au bouton cliqué
                this.classList.add('active');
                
                // Changer la disposition
                const view = this.dataset.view;
                if (view === 'list') {
                    machinesContainer.classList.add('list-view');
                    machinesContainer.classList.remove('grid-view');
                } else {
                    machinesContainer.classList.remove('list-view');
                    machinesContainer.classList.add('grid-view');
                }
            });
        });
    }
}

/**
 * Toggle des favoris sur les machines
 */
function initFavoriteToggle() {
    const favoriteBtns = document.querySelectorAll('.machine-favorite');
    
    favoriteBtns.forEach(btn => {
        btn.addEventListener('click', function(e) {
            e.preventDefault();
            e.stopPropagation();
            this.classList.toggle('active');
            
            // Animation de coeur
            this.style.transform = 'scale(1.2)';
            setTimeout(() => {
                this.style.transform = 'scale(1)';
            }, 200);
        });
    });
}

/**
 * Animations au scroll
 */
function initScrollAnimations() {
    const animatedElements = document.querySelectorAll('.machine-card, .stat-card, .step-card, .leaderboard-card');
    
    if (animatedElements.length > 0) {
        const observer = new IntersectionObserver((entries) => {
            entries.forEach(entry => {
                if (entry.isIntersecting) {
                    entry.target.classList.add('visible');
                }
            });
        }, {
            threshold: 0.1,
            rootMargin: '0px 0px -50px 0px'
        });
        
        animatedElements.forEach(el => {
            el.classList.add('fade-in');
            observer.observe(el);
        });
    }
}

/**
 * Fonctionnalité de recherche
 */

/**
 * Recherche globale du header — supprimée (S78, 2026-08-02).
 *
 * `.header-search` est désormais un vrai <form method="get"> vers `app_search`,
 * donc le navigateur fait le travail : Entrée soumet, le bouton soumet, et la
 * recherche fonctionne avant le chargement de ce fichier comme sans JS.
 *
 * ⚠️ Ne pas la réintroduire : l'ancienne version faisait
 * `button.setAttribute('type', 'button')`, ce qui **désarme** le bouton submit
 * du formulaire. Un helper qui rétablit ce comportement casse la recherche au
 * lieu de l'améliorer. L'URL était aussi codée en dur (`/search`), donc elle
 * ignorait `path()` et tout déploiement en sous-répertoire.
 */

/**
 * Affichage/masquage des champs mot de passe.
 */
function initPasswordToggles() {
    document.querySelectorAll('.password-toggle').forEach(button => {
        const group = button.closest('.input-wrapper, .form-group, .password-field') || button.parentElement;
        const input = group ? group.querySelector('input[type="password"], input[type="text"]') : null;

        if (!input) {
            return;
        }

        button.setAttribute('type', 'button');
        button.setAttribute('aria-label', 'Afficher le mot de passe');
        button.setAttribute('aria-pressed', 'false');

        button.addEventListener('click', () => {
            const shouldShow = input.type === 'password';
            input.type = shouldShow ? 'text' : 'password';
            button.setAttribute('aria-pressed', shouldShow ? 'true' : 'false');
            button.setAttribute('aria-label', shouldShow ? 'Masquer le mot de passe' : 'Afficher le mot de passe');
        });
    });
}

function initSearchFunctionality() {
    const searchInput = document.querySelector('.search-input-large');
    const searchSelect = document.querySelector('.search-select');
    const searchButton = document.querySelector('.search-section .btn-primary');
    
    if (searchInput) {
        // Recherche en temps réel (après 500ms de pause)
        let searchTimeout;
        searchInput.addEventListener('input', function() {
            clearTimeout(searchTimeout);
            searchTimeout = setTimeout(() => {
                performSearch(this.value, searchSelect ? searchSelect.value : 'all');
            }, 500);
        });
        
        // Recherche au clic sur le bouton
        if (searchButton) {
            searchButton.addEventListener('click', function() {
                performSearch(searchInput.value, searchSelect ? searchSelect.value : 'all');
            });
        }
        
        // Recherche avec la touche Entrée
        searchInput.addEventListener('keypress', function(e) {
            if (e.key === 'Enter') {
                performSearch(this.value, searchSelect ? searchSelect.value : 'all');
            }
        });
    }
}

/**
 * Effectuer une recherche (simulation pour la version statique)
 */
function performSearch(query, category) {
    if (!query || query.trim() === '') {
        // Si la recherche est vide, afficher tous les résultats
        const allCards = document.querySelectorAll('.machine-card');
        allCards.forEach(card => {
            card.style.display = '';
        });
        return;
    }
    
    const lowerQuery = query.toLowerCase();
    const machineCards = document.querySelectorAll('.machine-card');
    
    machineCards.forEach(card => {
        const name = card.querySelector('.machine-name')?.textContent.toLowerCase() || '';
        const description = card.querySelector('.machine-description')?.textContent.toLowerCase() || '';
        const location = card.querySelector('.machine-location')?.textContent.toLowerCase() || '';
        const badges = card.querySelectorAll('.badge-tag');
        
        let badgeText = '';
        badges.forEach(badge => {
            badgeText += badge.textContent.toLowerCase() + ' ';
        });
        
        const fullText = name + ' ' + description + ' ' + location + ' ' + badgeText;
        
        if (fullText.includes(lowerQuery)) {
            card.style.display = '';
        } else {
            card.style.display = 'none';
        }
    });
}

/**
 * Filtrer les machines par catégorie ou statut
 */
function initFilterFunctionality() {
    const filterSelects = document.querySelectorAll('.filter-select');
    
    if (filterSelects.length > 0) {
        filterSelects.forEach(select => {
            select.addEventListener('change', function() {
                applyFilters();
            });
        });
    }
}

/**
 * Appliquer les filtres
 */
function applyFilters() {
    const categorySelect = document.querySelector('.filter-select:nth-of-type(1)');
    const statusSelect = document.querySelector('.filter-select:nth-of-type(2)');
    
    const selectedCategory = categorySelect ? categorySelect.value : 'all';
    const selectedStatus = statusSelect ? statusSelect.value : 'all';
    
    const machineCards = document.querySelectorAll('.machine-card');
    
    machineCards.forEach(card => {
        const status = card.querySelector('.machine-status')?.textContent.toLowerCase() || '';
        const badges = card.querySelectorAll('.badge-tag');
        
        let hasCategory = false;
        badges.forEach(badge => {
            const badgeText = badge.textContent.toLowerCase();
            if (selectedCategory !== 'all' && badgeText.includes(selectedCategory)) {
                hasCategory = true;
            }
        });
        
        let categoryMatch = selectedCategory === 'all' || hasCategory;
        let statusMatch = selectedStatus === 'all' || 
                         (selectedStatus === 'disponible' && status.includes('disponible')) ||
                         (selectedStatus === 'maintenance' && status.includes('maintenance')) ||
                         (selectedStatus === 'panne' && status.includes('panne'));
        
        if (categoryMatch && statusMatch) {
            card.style.display = '';
        } else {
            card.style.display = 'none';
        }
    });
}

/**
 * Utilitaires
 */

// Debounce function pour les événements fréquents
function debounce(func, wait) {
    let timeout;
    return function executedFunction(...args) {
        const later = () => {
            clearTimeout(timeout);
            func(...args);
        };
        clearTimeout(timeout);
        timeout = setTimeout(later, wait);
    };
}

// Throttle function pour les événements de scroll
function throttle(func, limit) {
    let inThrottle;
    return function(...args) {
        if (!inThrottle) {
            func.apply(this, args);
            inThrottle = true;
            setTimeout(() => inThrottle = false, limit);
        }
    };
}

// Fonction pour formater les dates (si besoin plus tard)
function formatDate(dateString) {
    const options = { 
        year: 'numeric', 
        month: 'long', 
        day: 'numeric',
        hour: '2-digit',
        minute: '2-digit'
    };
    return new Date(dateString).toLocaleDateString(FABOS_LOCALE, options);
}

// Fonction pour formater la durée
function formatDuration(minutes) {
    if (minutes < 60) {
        return `${minutes} min`;
    }
    const hours = Math.floor(minutes / 60);
    const remainingMinutes = minutes % 60;
    if (remainingMinutes > 0) {
        return `${hours}h ${remainingMinutes}min`;
    }
    return `${hours}h`;
}

// Exporter pour utilisation dans d'autres scripts
if (typeof module !== 'undefined' && module.exports) {
    module.exports = {
        performSearch,
        applyFilters,
        formatDate,
        formatDuration,
        debounce,
        throttle
    };
}

/**
 * Récupère la préférence stockée en base et rend le thème cohérent sur
 * toutes les pages qui affichent le header connecté.
 */
function initThemePreference() {
    const marker = document.querySelector('[data-fabos-theme-preference]');
    if (!marker || !window.FabosTheme) {
        return;
    }

    window.FabosTheme.apply(marker.dataset.fabosThemePreference || 'light');
}

/**
 * Interrupteur jour/nuit de la page profil.
 * L'animation est immédiate ; la préférence est ensuite enregistrée sur
 * la route profil existante sans modifier les autres paramètres du compte.
 */
function initProfileThemeSwitch() {
    const button = document.querySelector('[data-theme-switch]');
    if (!button || !window.FabosTheme) {
        return;
    }

    const select = document.querySelector('[data-theme-select]');
    const label = document.querySelector('[data-theme-switch-label]');
    const saveUrl = button.dataset.saveUrl;
    const csrfToken = button.dataset.csrfToken;
    let requestSequence = 0;

    // ⚠️ S134c — les six libellés étaient écrits en français ici, sur une page que
    // FabOS sert aussi en quatre autres langues. Ils arrivent par `data-` depuis
    // `profil.html.twig`, comme les libellés de favoris de `machine-detail`. Ne pas
    // en réintroduire un en dur : ce fichier n'a accès à aucun catalogue.
    const labels = {
        toLight: button.dataset.labelToLight || '',
        toDark: button.dataset.labelToDark || '',
        systemDark: button.dataset.labelSystemDark || '',
        systemLight: button.dataset.labelSystemLight || '',
        modeDark: button.dataset.labelModeDark || '',
        modeLight: button.dataset.labelModeLight || '',
    };

    const updateControl = ({ preference, theme }) => {
        const isDark = theme === 'dark';
        button.setAttribute('aria-checked', isDark ? 'true' : 'false');
        button.setAttribute('aria-label', isDark ? labels.toLight : labels.toDark);
        button.dataset.visibleTheme = theme;

        if (label) {
            label.textContent = preference === 'system'
                ? (isDark ? labels.systemDark : labels.systemLight)
                : (isDark ? labels.modeDark : labels.modeLight);
        }
    };

    updateControl({
        preference: window.FabosTheme.getPreference(),
        theme: window.FabosTheme.getTheme()
    });

    document.addEventListener('fabos:themechange', event => updateControl(event.detail));

    if (select) {
        select.addEventListener('change', () => {
            window.FabosTheme.apply(select.value);
        });
    }

    button.addEventListener('click', async () => {
        if (button.classList.contains('is-saving')) {
            return;
        }

        const previousPreference = window.FabosTheme.getPreference();
        const nextPreference = window.FabosTheme.getTheme() === 'dark' ? 'light' : 'dark';
        const requestId = ++requestSequence;

        window.FabosTheme.apply(nextPreference);
        if (select) {
            select.value = nextPreference;
        }

        button.classList.add('is-saving');
        button.disabled = true;

        try {
            const body = new URLSearchParams({
                _token: csrfToken || '',
                _theme_only: '1',
                theme: nextPreference
            });

            const response = await fetch(saveUrl, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/x-www-form-urlencoded; charset=UTF-8',
                    'X-Requested-With': 'XMLHttpRequest',
                    'Accept': 'application/json'
                },
                body: body.toString(),
                credentials: 'same-origin'
            });

            const result = await response.json().catch(() => null);
            if (!response.ok || !result || result.ok !== true) {
                throw new Error(result && result.message ? result.message : 'Enregistrement impossible');
            }

            if (requestId === requestSequence) {
                button.classList.add('is-saved');
                window.setTimeout(() => button.classList.remove('is-saved'), 650);
            }
        } catch (error) {
            window.FabosTheme.apply(previousPreference);
            if (select) {
                select.value = previousPreference;
            }
            button.classList.add('has-error');
            if (label) {
                label.textContent = 'Enregistrement impossible';
            }
            window.setTimeout(() => {
                button.classList.remove('has-error');
                updateControl({
                    preference: window.FabosTheme.getPreference(),
                    theme: window.FabosTheme.getTheme()
                });
            }, 2200);
        } finally {
            button.classList.remove('is-saving');
            button.disabled = false;
        }
    });
}
