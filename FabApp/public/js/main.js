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
    initHeaderSearch();
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
        mobileNavToggle.addEventListener('click', function() {
            // Toggle l'affichage du menu
            mainNavbar.classList.toggle('active');
            
            // Changer l'icône du toggle
            this.classList.toggle('active');
        });
        
        // Fermer le menu quand on clique en dehors
        document.addEventListener('click', function(e) {
            if (!mobileNavToggle.contains(e.target) && !mainNavbar.contains(e.target)) {
                mainNavbar.classList.remove('active');
                mobileNavToggle.classList.remove('active');
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
 * Recherche globale du header.
 */
function initHeaderSearch() {
    const headerSearchBlocks = document.querySelectorAll('.header-search');

    headerSearchBlocks.forEach(block => {
        const input = block.querySelector('.search-input-header');
        const button = block.querySelector('.search-button-header');

        if (!input || !button) {
            return;
        }

        const submitSearch = () => {
            const query = input.value.trim();
            if (query === '') {
                input.focus();
                return;
            }

            const params = new URLSearchParams({ q: query });
            window.location.href = `/search?${params.toString()}`;
        };

        button.setAttribute('type', 'button');
        button.setAttribute('aria-label', 'Rechercher');
        button.addEventListener('click', submitSearch);
        input.addEventListener('keydown', event => {
            if (event.key === 'Enter') {
                event.preventDefault();
                submitSearch();
            }
        });
    });
}

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
    return new Date(dateString).toLocaleDateString('fr-FR', options);
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

    const updateControl = ({ preference, theme }) => {
        const isDark = theme === 'dark';
        button.setAttribute('aria-checked', isDark ? 'true' : 'false');
        button.setAttribute('aria-label', isDark ? 'Activer le thème clair' : 'Activer le thème sombre');
        button.dataset.visibleTheme = theme;

        if (label) {
            label.textContent = preference === 'system'
                ? `Système · ${isDark ? 'sombre' : 'clair'}`
                : (isDark ? 'Mode sombre' : 'Mode clair');
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
