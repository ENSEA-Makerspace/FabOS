(() => {
    'use strict';

    const root = document.querySelector('[data-section-journey]');
    if (!root) {
        return;
    }

    /*
     * S134c — the six sentences this file writes were French literals on a page
     * FabOS also serves in four other languages. Nothing reported it: a string
     * that is not a translation key is invisible to `debug:translation`, and the
     * template scanner strips <script> before counting. They ride in as JSON now.
     *
     * ⚠️ A new string belongs in `messages.*.yaml` and in the `journey-i18n` node
     * in `formation-detail.html.twig`. Never inline here.
     */
    let labels = {};
    const labelNode = document.getElementById('journey-i18n');
    if (labelNode) {
        try {
            labels = JSON.parse(labelNode.textContent || '{}');
        } catch (error) {
            labels = {};
        }
    }
    const t = (key) => (typeof labels[key] === 'string' ? labels[key] : key);
    const f = (key, values) => Object.entries(values).reduce(
        (text, [name, value]) => text.split(`%${name}%`).join(String(value)),
        t(key),
    );

    const cards = Array.from(root.querySelectorAll('[data-section-card]'));

    const stabilizeViewport = (mutate) => {
        const beforeTop = root.getBoundingClientRect().top;
        const beforeScroll = window.scrollY;
        mutate();
        window.requestAnimationFrame(() => {
            const afterTop = root.getBoundingClientRect().top;
            const delta = afterTop - beforeTop;
            if (Math.abs(delta) > 0.5) {
                window.scrollTo({ top: beforeScroll + delta, behavior: 'auto' });
            }
        });
    };

    const openCard = (card) => {
        if (!card || card.classList.contains('is-locked')) {
            return;
        }

        cards.forEach((candidate) => {
            const panel = candidate.querySelector('[data-section-panel]');
            const toggle = candidate.querySelector('[data-section-toggle]');
            const shouldOpen = candidate === card;
            candidate.classList.toggle('is-open', shouldOpen);
            if (panel) {
                panel.hidden = !shouldOpen;
            }
            if (toggle) {
                toggle.setAttribute('aria-expanded', shouldOpen ? 'true' : 'false');
            }
        });
    };

    cards.forEach((card) => {
        const toggle = card.querySelector('[data-section-toggle]');
        if (!toggle || toggle.disabled) {
            return;
        }
        toggle.addEventListener('click', () => {
            stabilizeViewport(() => {
                if (card.classList.contains('is-open')) {
                    const panel = card.querySelector('[data-section-panel]');
                    card.classList.remove('is-open');
                    toggle.setAttribute('aria-expanded', 'false');
                    if (panel) {
                        panel.hidden = true;
                    }
                    return;
                }
                openCard(card);
            });
        });
    });

    root.querySelectorAll('[data-section-quiz]').forEach((form) => {
        form.addEventListener('submit', async (event) => {
            event.preventDefault();

            const message = form.querySelector('[data-section-quiz-message]');
            const submit = form.querySelector('[data-section-quiz-submit]');
            const questionBlocks = Array.from(form.querySelectorAll('[data-section-question]'));
            const answers = {};
            let complete = true;

            questionBlocks.forEach((block) => {
                const questionId = block.dataset.questionId;
                const selected = Array.from(block.querySelectorAll('input:checked')).map((input) => input.value);
                if (!questionId || selected.length === 0) {
                    complete = false;
                    block.classList.add('has-error');
                } else {
                    block.classList.remove('has-error');
                    answers[questionId] = selected;
                }
            });

            if (!complete) {
                if (message) {
                    message.textContent = t('js_answer_all');
                    message.className = 'section-mini-quiz__message is-error';
                }
                return;
            }

            if (submit) {
                submit.disabled = true;
                submit.dataset.originalLabel = submit.textContent;
                submit.textContent = t('js_checking');
            }
            if (message) {
                message.textContent = t('js_server_checks');
                message.className = 'section-mini-quiz__message';
            }

            try {
                const response = await fetch(form.dataset.saveUrl || '', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json',
                        'Accept': 'application/json',
                    },
                    body: JSON.stringify({
                        _token: form.dataset.csrfToken || '',
                        answers,
                    }),
                });
                const payload = await response.json().catch(() => ({}));

                if (!response.ok || !payload.ok) {
                    throw new Error(payload.message || t('js_failed'));
                }

                const result = payload.result || {};
                if (result.completed) {
                    if (message) {
                        message.textContent = t('js_section_validated');
                        message.className = 'section-mini-quiz__message is-success';
                    }
                    const card = form.closest('[data-section-card]');
                    if (card) {
                        card.classList.remove('is-available');
                        card.classList.add('is-completed');
                    }
                    window.setTimeout(() => window.location.reload(), 850);
                    return;
                }

                if (message) {
                    message.textContent = f('js_score_retry', { score: result.attemptScore ?? 0 });
                    message.className = 'section-mini-quiz__message is-error';
                }
            } catch (error) {
                if (message) {
                    message.textContent = error instanceof Error ? error.message : t('js_failed');
                    message.className = 'section-mini-quiz__message is-error';
                }
            } finally {
                if (submit) {
                    submit.disabled = false;
                    submit.textContent = submit.dataset.originalLabel || 'Valider la section';
                }
            }
        });
    });
})();
