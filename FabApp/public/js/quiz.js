(() => {
    'use strict';

    const app = document.querySelector('[data-quiz-app]');
    const dataNode = document.getElementById('quiz-data');
    if (!app || !dataNode) {
        return;
    }

    let quiz;
    try {
        quiz = JSON.parse(dataNode.textContent || '{}');
    } catch (error) {
        console.error('Impossible de lire les données du quiz.', error);
        return;
    }

    if (!Array.isArray(quiz.questions) || quiz.questions.length === 0) {
        return;
    }

    const persistence = {
        enabled: app.dataset.persistenceEnabled === '1',
        saveUrl: app.dataset.saveUrl || '',
        csrfToken: app.dataset.csrfToken || '',
    };

    const elements = {
        progressPercent: app.querySelector('[data-progress-percent]'),
        progressBar: app.querySelector('[data-progress-bar]'),
        answeredCount: app.querySelector('[data-answered-count]'),
        stepList: app.querySelector('[data-step-list]'),
        questionCounter: app.querySelector('[data-question-counter]'),
        questionType: app.querySelector('[data-question-type]'),
        questionText: app.querySelector('[data-question-text]'),
        questionInstruction: app.querySelector('[data-question-instruction]'),
        options: app.querySelector('[data-options]'),
        inlineMessage: app.querySelector('[data-inline-message]'),
        previous: app.querySelector('[data-previous]'),
        next: app.querySelector('[data-next]'),
        navigation: app.querySelector('[data-navigation]'),
        questionView: app.querySelector('[data-question-view]'),
        result: app.querySelector('[data-result]'),
        resultActions: app.querySelector('[data-result-actions]'),
        resultScore: app.querySelector('[data-result-score]'),
        resultRing: app.querySelector('[data-result-ring]'),
        resultEyebrow: app.querySelector('[data-result-eyebrow]'),
        resultTitle: app.querySelector('[data-result-title]'),
        resultMessage: app.querySelector('[data-result-message]'),
        saveStatus: app.querySelector('[data-save-status]'),
        correctCount: app.querySelector('[data-correct-count]'),
        wrongCount: app.querySelector('[data-wrong-count]'),
        review: app.querySelector('[data-review]'),
        restart: app.querySelector('[data-restart]'),
    };

    const state = {
        currentIndex: 0,
        answers: new Map(),
        finished: false,
        saving: false,
    };

    const normalizeIds = (values) => [...values].map(String).sort();

    const selectedFor = (question) => state.answers.get(String(question.id)) || [];

    const answeredCount = () => {
        let count = 0;
        quiz.questions.forEach((question) => {
            if (selectedFor(question).length > 0) {
                count += 1;
            }
        });
        return count;
    };

    const renderSteps = () => {
        elements.stepList.textContent = '';

        quiz.questions.forEach((question, index) => {
            const button = document.createElement('button');
            button.type = 'button';
            button.className = 'quiz-step';
            if (index === state.currentIndex && !state.finished) {
                button.classList.add('is-active');
            }
            if (selectedFor(question).length > 0) {
                button.classList.add('is-answered');
            }
            button.setAttribute('aria-label', `Aller à la question ${index + 1}`);

            const number = document.createElement('span');
            number.className = 'quiz-step__number';
            number.textContent = String(index + 1).padStart(2, '0');

            const label = document.createElement('span');
            label.className = 'quiz-step__label';
            label.textContent = question.text;

            const status = document.createElement('span');
            status.className = 'quiz-step__state';
            status.setAttribute('aria-hidden', 'true');

            button.append(number, label, status);
            button.addEventListener('click', () => {
                if (state.finished) {
                    return;
                }
                state.currentIndex = index;
                clearMessage();
                render();
            });

            elements.stepList.appendChild(button);
        });
    };

    const updateProgress = () => {
        const count = answeredCount();
        const percent = Math.round((count / quiz.questions.length) * 100);
        elements.progressPercent.textContent = `${percent} %`;
        elements.progressBar.style.width = `${percent}%`;
        elements.answeredCount.textContent = String(count);
    };

    const clearMessage = () => {
        elements.inlineMessage.textContent = '';
    };

    const setMessage = (message) => {
        elements.inlineMessage.textContent = message;
    };

    const setAnswer = (question, choiceId, checked) => {
        const questionId = String(question.id);
        const current = new Set(selectedFor(question));

        if (question.type === 'multiple') {
            if (checked) {
                current.add(String(choiceId));
            } else {
                current.delete(String(choiceId));
            }
            state.answers.set(questionId, normalizeIds(current));
        } else {
            state.answers.set(questionId, [String(choiceId)]);
        }

        clearMessage();
        renderQuestionOptions(question);
        renderSteps();
        updateProgress();
    };

    const renderQuestionOptions = (question) => {
        elements.options.textContent = '';
        const selected = new Set(selectedFor(question));
        const multiple = question.type === 'multiple';

        question.choices.forEach((choice, index) => {
            const label = document.createElement('label');
            label.className = 'quiz-option';
            label.dataset.optionType = multiple ? 'multiple' : 'single';
            if (selected.has(String(choice.id))) {
                label.classList.add('is-selected');
            }

            const input = document.createElement('input');
            input.type = multiple ? 'checkbox' : 'radio';
            input.name = `question-${question.id}`;
            input.value = String(choice.id);
            input.checked = selected.has(String(choice.id));
            input.addEventListener('change', () => setAnswer(question, choice.id, input.checked));

            const marker = document.createElement('span');
            marker.className = 'quiz-option__marker';
            marker.textContent = input.checked ? '✓' : String.fromCharCode(65 + index);

            const text = document.createElement('span');
            text.className = 'quiz-option__text';
            text.textContent = choice.text;

            label.append(input, marker, text);
            elements.options.appendChild(label);
        });
    };

    const renderQuestion = () => {
        const question = quiz.questions[state.currentIndex];
        const multiple = question.type === 'multiple';

        elements.questionCounter.textContent = `Question ${state.currentIndex + 1} sur ${quiz.questions.length}`;
        elements.questionType.textContent = multiple ? 'Choix multiples' : 'Choix unique';
        elements.questionText.textContent = question.text;
        elements.questionInstruction.textContent = multiple
            ? 'Plusieurs réponses peuvent être correctes.'
            : 'Sélectionnez une seule réponse.';

        renderQuestionOptions(question);

        elements.previous.disabled = state.currentIndex === 0;
        elements.next.textContent = '';
        const nextLabel = document.createTextNode(state.currentIndex === quiz.questions.length - 1 ? 'Voir mon résultat' : 'Suivant');
        const arrow = document.createElementNS('http://www.w3.org/2000/svg', 'svg');
        arrow.setAttribute('viewBox', '0 0 24 24');
        arrow.setAttribute('aria-hidden', 'true');
        const path = document.createElementNS('http://www.w3.org/2000/svg', 'path');
        path.setAttribute('d', 'M9 18l6-6-6-6');
        arrow.appendChild(path);
        elements.next.append(nextLabel, arrow);
    };

    const isAnswered = (question) => selectedFor(question).length > 0;

    const questionIsCorrect = (question) => {
        const expected = normalizeIds(question.choices.filter((choice) => choice.correct).map((choice) => choice.id));
        const actual = normalizeIds(selectedFor(question));
        return expected.length === actual.length && expected.every((value, index) => value === actual[index]);
    };

    const formatAnswer = (question, ids) => {
        const selected = new Set(normalizeIds(ids));
        const texts = question.choices
            .filter((choice) => selected.has(String(choice.id)))
            .map((choice) => choice.text);
        return texts.length > 0 ? texts.join(' · ') : 'Aucune réponse';
    };

    const setSaveStatus = (message, type = '') => {
        if (!elements.saveStatus) {
            return;
        }
        elements.saveStatus.textContent = message;
        elements.saveStatus.classList.toggle('is-success', type === 'success');
        elements.saveStatus.classList.toggle('is-error', type === 'error');
    };

    const serializeAnswers = () => {
        const answers = {};
        quiz.questions.forEach((question) => {
            answers[String(question.id)] = selectedFor(question);
        });
        return answers;
    };

    const saveResult = async () => {
        if (!persistence.enabled || !persistence.saveUrl) {
            setSaveStatus('Résultat calculé localement. Connectez-vous et utilisez un quiz persistant pour l’enregistrer.');
            return null;
        }

        state.saving = true;
        setSaveStatus('Enregistrement du résultat…');

        try {
            const response = await fetch(persistence.saveUrl, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                    'Accept': 'application/json',
                    'X-Requested-With': 'XMLHttpRequest',
                },
                credentials: 'same-origin',
                body: JSON.stringify({
                    _token: persistence.csrfToken,
                    answers: serializeAnswers(),
                }),
            });

            const payload = await response.json().catch(() => ({}));
            if (!response.ok || payload.ok !== true) {
                throw new Error(payload.message || 'Impossible d’enregistrer le résultat.');
            }

            setSaveStatus(payload.message || 'Résultat enregistré.', 'success');
            return payload.result || null;
        } catch (error) {
            setSaveStatus(error instanceof Error ? error.message : 'Impossible d’enregistrer le résultat.', 'error');
            return null;
        } finally {
            state.saving = false;
        }
    };

    const resolveBadgeImage = (badge) => {
        const lookup = `${badge?.nom ?? ''} ${badge?.icone ?? ''}`.toLowerCase();
        if (lookup.includes('ultimaker')) return '/images/badges/formations/ultimaker-s5.png';
        if (lookup.includes('prusa') || lookup.includes('mk3')) return '/images/badges/formations/prusa-mk3s.png';
        if (lookup.includes('laser') || lookup.includes('co2')) return '/images/badges/formations/laser-co2.png';
        if (lookup.includes('soudure') || lookup.includes('solder')) return '/images/badges/formations/soudure-electronique.png';
        if (lookup.includes('vinyl') || lookup.includes('vinyle')) return '/images/badges/formations/decoupe-vinyle.png';
        if (lookup.includes('cnc') || lookup.includes('fraiseuse') || lookup.includes('usinage')) return '/images/badges/formations/fraiseuse-cnc.png';
        if (lookup.includes('oscillo')) return '/images/badges/formations/oscilloscope.png';
        if (lookup.includes('brod') || lookup.includes('textile')) return '/images/badges/formations/brodeuse-numerique.png';
        return '/images/badges/formations/impression-3d.png';
    };

    const showBadgeUnlock = (badge) => {
        if (!badge) {
            return;
        }

        const toast = document.createElement('aside');
        toast.className = 'quiz-badge-unlock-toast';
        toast.setAttribute('role', 'status');
        toast.innerHTML = `
            <img src="${resolveBadgeImage(badge)}" alt="" class="quiz-badge-unlock-toast__image">
            <div>
                <strong>Badge débloqué !</strong>
                <span>${String(badge.nom ?? 'Nouveau badge')}</span>
            </div>
            <a href="/profil#badges">Voir le profil</a>
        `;
        document.body.appendChild(toast);
        requestAnimationFrame(() => toast.classList.add('is-visible'));
        window.setTimeout(() => {
            toast.classList.remove('is-visible');
            window.setTimeout(() => toast.remove(), 260);
        }, 6500);
    };

    const finishQuiz = async () => {
        const firstMissing = quiz.questions.findIndex((question) => !isAnswered(question));
        if (firstMissing !== -1) {
            state.currentIndex = firstMissing;
            render();
            setMessage('Répondez à cette question avant de valider le quiz.');
            return;
        }

        state.finished = true;
        const correct = quiz.questions.filter(questionIsCorrect).length;
        const wrong = quiz.questions.length - correct;
        const score = Math.round((correct / quiz.questions.length) * 100);
        const passed = score >= Number(quiz.passingScore || 0);

        elements.questionView.hidden = true;
        elements.navigation.hidden = true;
        elements.result.hidden = false;
        elements.resultActions.hidden = false;
        elements.result.classList.toggle('is-success', passed);
        elements.resultScore.textContent = `${score} %`;
        elements.resultRing.style.setProperty('--score-angle', `${score * 3.6}deg`);
        elements.resultEyebrow.textContent = passed ? 'Quiz validé' : 'Quiz à revoir';
        elements.resultTitle.textContent = passed ? 'Bravo, les règles essentielles sont acquises.' : 'Quelques points doivent être revus.';
        elements.resultMessage.textContent = passed
            ? 'Votre résultat atteint le seuil demandé. Il va maintenant être vérifié et enregistré par le serveur.'
            : 'Relisez les réponses indiquées ci-dessous. Votre tentative sera enregistrée et vous pourrez recommencer.';
        elements.correctCount.textContent = String(correct);
        elements.wrongCount.textContent = String(wrong);

        const serverResult = await saveResult();
        if (serverResult) {
            const serverScore = Number(serverResult.attemptScore ?? score);
            const serverPassed = Boolean(serverResult.passed);
            elements.resultScore.textContent = `${serverScore} %`;
            elements.resultRing.style.setProperty('--score-angle', `${serverScore * 3.6}deg`);
            elements.result.classList.toggle('is-success', serverPassed);
            elements.resultEyebrow.textContent = serverPassed ? 'Quiz validé' : 'Quiz à revoir';
            elements.resultTitle.textContent = serverPassed
                ? 'Bravo, votre résultat est enregistré.'
                : 'Résultat enregistré, quelques points restent à revoir.';
            elements.correctCount.textContent = String(serverResult.correctCount ?? correct);
            elements.wrongCount.textContent = String((serverResult.questionCount ?? quiz.questions.length) - (serverResult.correctCount ?? correct));
            if (serverResult.badgeAwarded) {
                showBadgeUnlock(serverResult.badge);
            }
        }

        renderReview();
        renderSteps();
        updateProgress();
        elements.result.scrollIntoView({ behavior: 'smooth', block: 'start' });
    };

    const renderReview = () => {
        elements.review.textContent = '';

        quiz.questions.forEach((question, index) => {
            const correct = questionIsCorrect(question);
            const expectedIds = question.choices.filter((choice) => choice.correct).map((choice) => choice.id);
            const item = document.createElement('article');
            item.className = `quiz-review-item${correct ? ' is-correct' : ''}`;

            const title = document.createElement('strong');
            title.textContent = `${correct ? '✓' : '×'} Question ${index + 1} — ${correct ? 'Correct' : 'À revoir'}`;

            const answer = document.createElement('p');
            answer.textContent = `Votre réponse : ${formatAnswer(question, selectedFor(question))}`;

            item.append(title, answer);

            if (!correct) {
                const expected = document.createElement('p');
                expected.textContent = `Réponse attendue : ${formatAnswer(question, expectedIds)}`;
                item.appendChild(expected);
            }

            elements.review.appendChild(item);
        });
    };

    const restart = () => {
        state.currentIndex = 0;
        state.answers.clear();
        state.finished = false;
        elements.questionView.hidden = false;
        elements.navigation.hidden = false;
        elements.result.hidden = true;
        elements.resultActions.hidden = true;
        elements.result.classList.remove('is-success');
        setSaveStatus('');
        clearMessage();
        render();
        app.scrollIntoView({ behavior: 'smooth', block: 'start' });
    };

    const goNext = () => {
        if (state.saving) {
            return;
        }
        const question = quiz.questions[state.currentIndex];
        if (!isAnswered(question)) {
            setMessage('Sélectionnez au moins une réponse pour continuer.');
            return;
        }

        if (state.currentIndex === quiz.questions.length - 1) {
            finishQuiz();
            return;
        }

        state.currentIndex += 1;
        clearMessage();
        render();
    };

    const goPrevious = () => {
        if (state.currentIndex === 0) {
            return;
        }
        state.currentIndex -= 1;
        clearMessage();
        render();
    };

    const render = () => {
        if (!state.finished) {
            renderQuestion();
        }
        renderSteps();
        updateProgress();
    };

    elements.next.addEventListener('click', goNext);
    elements.previous.addEventListener('click', goPrevious);
    elements.restart.addEventListener('click', restart);

    document.addEventListener('keydown', (event) => {
        if (state.finished || event.target instanceof HTMLInputElement || event.target instanceof HTMLTextAreaElement) {
            return;
        }
        if (event.key === 'ArrowLeft') {
            goPrevious();
        } else if (event.key === 'ArrowRight') {
            goNext();
        }
    });

    render();
})();
