(() => {
    'use strict';

    const root = document.getElementById('fabos-calendar-app');
    const dataNode = document.getElementById('fabos-calendar-data');

    if (!root || !dataNode) {
        return;
    }

    let payload;
    try {
        payload = JSON.parse(dataNode.textContent || '{}');
    } catch (error) {
        console.error('Impossible de charger les données du calendrier FabOS.', error);
        return;
    }

    const config = {
        startHour: Number(payload.config?.startHour ?? 8),
        endHour: Number(payload.config?.endHour ?? 20),
        slotMinutes: Number(payload.config?.slotMinutes ?? 30),
        userId: payload.config?.userId === null ? null : Number(payload.config?.userId),
        isAuthenticated: Boolean(payload.config?.isAuthenticated),
        reservationUrl: String(payload.config?.reservationUrl ?? ''),
        loginUrl: String(payload.config?.loginUrl ?? '/login'),
        formationsUrl: String(payload.config?.formationsUrl ?? '/formations')
    };

    const machines = Array.isArray(payload.machines)
        ? payload.machines.map(machine => ({
            id: Number(machine.id),
            name: String(machine.name ?? 'Machine'),
            status: String(machine.status ?? 'idle'),
            granularity: Math.max(1, Number(machine.granularity ?? 60)),
            authorized: Boolean(machine.authorized),
            authorizationStatus: String(machine.authorizationStatus ?? 'missing_badge'),
            missingBadges: Array.isArray(machine.missingBadges)
                ? machine.missingBadges.map(name => String(name))
                : []
        }))
        : [];

    const reservations = Array.isArray(payload.reservations)
        ? payload.reservations
            .filter(reservation => String(reservation.status ?? 'confirmed').toLowerCase() !== 'cancelled')
            .map(reservation => ({
                id: Number(reservation.id),
                machineId: Number(reservation.machineId),
                date: String(reservation.date ?? ''),
                start: String(reservation.start ?? '00:00'),
                end: String(reservation.end ?? '00:00'),
                userId: reservation.userId === null ? null : Number(reservation.userId),
                userName: String(reservation.userName ?? 'Utilisateur'),
                motif: reservation.motif ? String(reservation.motif) : '',
                status: String(reservation.status ?? 'confirmed')
            }))
        : [];

    const openingHours = new Map();
    (Array.isArray(payload.openingHours) ? payload.openingHours : []).forEach(row => {
        openingHours.set(Number(row.dayIndex), {
            isClosed: Boolean(row.isClosed),
            openMinutes: timeToMinutes(row.openTime),
            closeMinutes: timeToMinutes(row.closeTime),
            label: String(row.label ?? '')
        });
    });

    const machineMap = new Map(machines.map(machine => [machine.id, machine]));

    const elements = {
        grid: document.getElementById('calendar-grid'),
        daysHeader: document.getElementById('days-header'),
        currentWeek: document.getElementById('current-week-display'),
        prevWeek: document.getElementById('prev-week'),
        nextWeek: document.getElementById('next-week'),
        todayWeek: document.getElementById('today-week'),
        scroll: document.getElementById('calendar-scroll'),
        filterCheckboxes: Array.from(document.querySelectorAll('.machine-filter-checkbox')),
        selectAll: document.getElementById('select-all-machines'),
        clearAll: document.getElementById('clear-all-machines'),
        selectedMachinesCount: document.getElementById('selected-machines-count'),
        visibleMachinesCount: document.getElementById('visible-machines-count'),
        weekReservationsCount: document.getElementById('week-reservations-count'),
        openDaysCount: document.getElementById('open-days-count'),
        emptyState: document.getElementById('calendar-empty-state'),
        machineSelect: document.getElementById('reservation-machine'),
        startInput: document.getElementById('reservation-date-debut'),
        endInput: document.getElementById('reservation-date-fin'),
        reasonInput: document.getElementById('reservation-motif'),
        form: document.getElementById('reservation-form'),
        submit: document.getElementById('reservation-submit'),
        formMessage: document.getElementById('reservation-message'),
        selectionPill: document.getElementById('selection-state-pill'),
        selectionSummary: document.getElementById('booking-selection-summary'),
        selectionTitle: document.getElementById('booking-selection-title'),
        selectionDetails: document.getElementById('booking-selection-details'),
        clearSelection: document.getElementById('clear-selection'),
        toastRegion: document.getElementById('calendar-toast-region')
    };

    const state = {
        currentDate: startOfDay(new Date()),
        selectedMachineIds: new Set(machines.map(machine => machine.id)),
        selectedSlot: null,
        selectedEventId: null,
        initialScrollDone: false
    };

    init();

    function init() {
        bindEvents();
        initializeMachineSelect();
        initializeInputLimits();
        render();
    }

    function bindEvents() {
        elements.prevWeek?.addEventListener('click', () => changeWeek(-1));
        elements.nextWeek?.addEventListener('click', () => changeWeek(1));
        elements.todayWeek?.addEventListener('click', () => {
            state.currentDate = startOfDay(new Date());
            state.initialScrollDone = false;
            clearSelection({ silent: true });
            render();
        });

        elements.filterCheckboxes.forEach(checkbox => {
            checkbox.addEventListener('change', () => {
                const machineId = Number(checkbox.dataset.machineId);
                if (checkbox.checked) {
                    state.selectedMachineIds.add(machineId);
                } else {
                    state.selectedMachineIds.delete(machineId);
                }
                render();
            });
        });

        elements.selectAll?.addEventListener('click', () => {
            elements.filterCheckboxes.forEach(checkbox => {
                checkbox.checked = true;
                state.selectedMachineIds.add(Number(checkbox.dataset.machineId));
            });
            render();
        });

        elements.clearAll?.addEventListener('click', () => {
            elements.filterCheckboxes.forEach(checkbox => {
                checkbox.checked = false;
            });
            state.selectedMachineIds.clear();
            render();
        });

        elements.machineSelect?.addEventListener('change', () => {
            const machineId = Number(elements.machineSelect.value);
            if (machineId > 0) {
                ensureMachineVisible(machineId);
                refreshMachineAvailabilityMessage(machineId);
            }
        });

        elements.clearSelection?.addEventListener('click', () => clearSelection());
        elements.form?.addEventListener('submit', submitReservation);
    }

    function initializeMachineSelect() {
        if (!elements.machineSelect || elements.machineSelect.value) {
            return;
        }

        const preferred = machines.find(machine =>
            classifyMachineStatus(machine.status) === 'available' && machine.authorized
        ) ?? null;
        if (preferred) {
            elements.machineSelect.value = String(preferred.id);
        }
    }

    function initializeInputLimits() {
        const now = new Date();
        const minValue = formatDateTimeLocal(roundDateUp(now, config.slotMinutes));
        if (elements.startInput) {
            elements.startInput.min = minValue;
            elements.startInput.step = String(config.slotMinutes * 60);
        }
        if (elements.endInput) {
            elements.endInput.min = minValue;
            elements.endInput.step = String(config.slotMinutes * 60);
        }
    }

    function changeWeek(offset) {
        const next = new Date(state.currentDate);
        next.setDate(next.getDate() + offset * 7);
        state.currentDate = next;
        clearSelection({ silent: true });
        render();
        elements.scroll?.scrollTo({ left: 0, behavior: 'smooth' });
    }

    function render() {
        const weekDays = getWeekDays(state.currentDate);
        const visibleReservations = getVisibleWeekReservations(weekDays);

        renderWeekHeading(weekDays);
        renderDaysHeader(weekDays);
        renderGrid(weekDays, visibleReservations);
        renderStats(visibleReservations);
        renderEmptyState(visibleReservations);

        if (!state.initialScrollDone) {
            requestAnimationFrame(() => scrollToRelevantDay(weekDays));
            state.initialScrollDone = true;
        }
    }

    function renderWeekHeading(weekDays) {
        if (!elements.currentWeek) {
            return;
        }

        const first = weekDays[0];
        const last = weekDays[6];
        const firstText = first.toLocaleDateString('fr-FR', { day: 'numeric', month: 'long' });
        const lastText = last.toLocaleDateString('fr-FR', { day: 'numeric', month: 'long', year: 'numeric' });
        elements.currentWeek.textContent = `Semaine ${getISOWeek(first)} · ${firstText} – ${lastText}`;
    }

    function renderDaysHeader(weekDays) {
        if (!elements.daysHeader) {
            return;
        }

        const fragment = document.createDocumentFragment();
        const timeHeader = document.createElement('div');
        timeHeader.className = 'calendar-time-header';
        timeHeader.textContent = 'Heure';
        fragment.appendChild(timeHeader);

        const today = startOfDay(new Date());
        weekDays.forEach(day => {
            const header = document.createElement('div');
            header.className = 'calendar-day-header';
            if (isSameDay(day, today)) {
                header.classList.add('is-today');
            }

            header.innerHTML = `
                <span class="calendar-day-header__weekday">${escapeHtml(day.toLocaleDateString('fr-FR', { weekday: 'long' }))}</span>
                <span class="calendar-day-header__number">${day.getDate()}</span>
                <span class="calendar-day-header__month">${escapeHtml(day.toLocaleDateString('fr-FR', { month: 'short' }))}</span>
            `;
            fragment.appendChild(header);
        });

        elements.daysHeader.replaceChildren(fragment);
    }

    function renderGrid(weekDays, visibleReservations) {
        if (!elements.grid) {
            return;
        }

        const displayStart = config.startHour * 60;
        const displayEnd = config.endHour * 60;
        const slotCount = Math.max(1, Math.ceil((displayEnd - displayStart) / config.slotMinutes));
        const fragment = document.createDocumentFragment();
        const now = new Date();

        for (let slotIndex = 0; slotIndex < slotCount; slotIndex += 1) {
            const slotMinutes = displayStart + slotIndex * config.slotMinutes;
            const row = slotIndex + 1;
            const timeCell = document.createElement('div');
            timeCell.className = 'calendar-time-cell';
            timeCell.style.gridRow = String(row);
            timeCell.style.gridColumn = '1';

            if (slotMinutes % 60 === 0) {
                timeCell.classList.add('is-hour-start');
                timeCell.textContent = minutesToTime(slotMinutes);
            } else {
                timeCell.classList.add('is-half-hour');
                timeCell.textContent = '30';
            }
            fragment.appendChild(timeCell);

            weekDays.forEach((day, dayIndex) => {
                const slotStart = dateWithMinutes(day, slotMinutes);
                const slotEnd = dateWithMinutes(day, slotMinutes + config.slotMinutes);
                const cell = document.createElement('button');
                cell.type = 'button';
                cell.className = 'calendar-slot-cell';
                cell.style.gridRow = String(row);
                cell.style.gridColumn = String(dayIndex + 2);
                cell.dataset.date = formatDateKey(day);
                cell.dataset.minutes = String(slotMinutes);
                cell.setAttribute('role', 'gridcell');

                if (slotMinutes % 60 === 0) {
                    cell.classList.add('is-hour-start');
                }

                const isPast = slotStart <= now;
                const isOpen = isPeriodWithinOpening(day, slotMinutes, slotMinutes + config.slotMinutes);

                if (isPast) {
                    cell.classList.add('is-past');
                    cell.setAttribute('aria-disabled', 'true');
                    cell.setAttribute('aria-label', `${formatReadableDate(day)} à ${minutesToTime(slotMinutes)}, créneau passé`);
                } else if (!isOpen) {
                    cell.classList.add('is-closed');
                    cell.setAttribute('aria-disabled', 'true');
                    cell.setAttribute('aria-label', `${formatReadableDate(day)} à ${minutesToTime(slotMinutes)}, FabLab fermé`);
                } else {
                    cell.classList.add('is-open');
                    cell.setAttribute('aria-label', `${formatReadableDate(day)} à ${minutesToTime(slotMinutes)}, créneau disponible à sélectionner`);
                }

                if (state.selectedSlot && state.selectedSlot.date === formatDateKey(day)) {
                    const selectionStart = state.selectedSlot.startMinutes;
                    const selectionEnd = state.selectedSlot.endMinutes;
                    if (slotMinutes >= selectionStart && slotMinutes < selectionEnd) {
                        cell.classList.add('is-selected');
                    }
                }

                cell.addEventListener('click', () => handleSlotClick(cell, day, slotMinutes));
                fragment.appendChild(cell);
            });
        }

        const laidOutReservations = assignOverlapLanes(visibleReservations);
        laidOutReservations.forEach(reservation => {
            const machine = machineMap.get(reservation.machineId);
            if (!machine) {
                return;
            }

            const dayIndex = weekDays.findIndex(day => formatDateKey(day) === reservation.date);
            if (dayIndex < 0) {
                return;
            }

            const rawStart = timeToMinutes(reservation.start);
            const rawEnd = timeToMinutes(reservation.end);
            const start = Math.max(displayStart, rawStart);
            const end = Math.min(displayEnd, rawEnd);
            if (end <= start) {
                return;
            }

            const startRow = Math.floor((start - displayStart) / config.slotMinutes) + 1;
            const rowSpan = Math.max(1, Math.ceil((end - start) / config.slotMinutes));
            const event = document.createElement('button');
            event.type = 'button';
            event.className = 'calendar-event';
            event.style.gridRow = `${startRow} / span ${rowSpan}`;
            event.style.gridColumn = String(dayIndex + 2);
            event.style.setProperty('--lane-index', String(reservation.laneIndex ?? 0));
            event.style.setProperty('--lane-count', String(reservation.laneCount ?? 1));
            event.dataset.reservationId = String(reservation.id);

            const statusClass = classifyMachineStatus(machine.status);
            if (statusClass === 'maintenance') {
                event.classList.add('is-maintenance');
            } else if (statusClass === 'broken') {
                event.classList.add('is-broken');
            } else if (config.userId !== null && reservation.userId === config.userId) {
                event.classList.add('is-mine');
            }

            if (state.selectedEventId === reservation.id) {
                event.classList.add('is-selected');
            }

            const accessibleOwner = config.userId !== null && reservation.userId === config.userId
                ? 'votre réservation'
                : `réservation de ${reservation.userName}`;
            event.setAttribute(
                'aria-label',
                `${machine.name}, ${reservation.start} à ${reservation.end}, ${accessibleOwner}`
            );

            event.innerHTML = `
                <span class="calendar-event__card">
                    <span class="calendar-event__machine">${escapeHtml(machine.name)}</span>
                    <span class="calendar-event__time">${escapeHtml(reservation.start)} – ${escapeHtml(reservation.end)}</span>
                    <span class="calendar-event__user">${escapeHtml(reservation.userName)}</span>
                </span>
            `;
            event.addEventListener('click', () => selectReservationEvent(reservation, machine));
            fragment.appendChild(event);
        });

        elements.grid.replaceChildren(fragment);
    }

    function handleSlotClick(cell, day, startMinutes) {
        if (cell.classList.contains('is-past')) {
            showToast('Créneau passé', 'Choisissez un créneau qui commence dans le futur.', 'error');
            return;
        }

        if (cell.classList.contains('is-closed')) {
            showToast('FabLab fermé', 'Ce créneau est en dehors des horaires d’ouverture.', 'error');
            return;
        }

        if (!config.isAuthenticated) {
            showToast('Connexion requise', 'Connectez-vous pour créer une réservation.', 'info');
            return;
        }

        const machine = getSelectedBookingMachine();
        if (!machine) {
            showToast('Machine nécessaire', 'Sélectionnez d’abord une machine dans le formulaire.', 'error');
            elements.machineSelect?.focus();
            return;
        }

        if (!machine.authorized) {
            const missing = machine.missingBadges.length > 0
                ? ` Badge manquant : ${machine.missingBadges.join(', ')}.`
                : '';
            showToast('Formation requise', `Vous devez valider la formation avant de réserver ${machine.name}.${missing}`, 'error');
            setFormMessage(`Formation nécessaire avant réservation.${missing}`, 'error');
            return;
        }

        const machineStatus = classifyMachineStatus(machine.status);
        if (machineStatus === 'maintenance' || machineStatus === 'broken') {
            const statusText = machineStatus === 'maintenance' ? 'en maintenance' : 'en panne';
            showToast('Machine indisponible', `${machine.name} est actuellement ${statusText}.`, 'error');
            return;
        }

        const duration = Math.max(config.slotMinutes, machine.granularity || 60);
        const endMinutes = startMinutes + duration;
        const dateKey = formatDateKey(day);
        const startDate = dateWithMinutes(day, startMinutes);
        const endDate = dateWithMinutes(day, endMinutes);

        if (!isPeriodWithinOpening(day, startMinutes, endMinutes)) {
            showToast('Horaire non disponible', 'La durée de réservation dépasse l’heure de fermeture.', 'error');
            return;
        }

        if (hasOverlap(machine.id, startDate, endDate)) {
            showToast('Créneau déjà réservé', `${machine.name} est déjà réservée sur cette période.`, 'error');
            return;
        }

        state.selectedEventId = null;
        state.selectedSlot = {
            date: dateKey,
            startMinutes,
            endMinutes,
            machineId: machine.id
        };

        if (elements.startInput) {
            elements.startInput.value = formatDateTimeLocal(startDate);
        }
        if (elements.endInput) {
            elements.endInput.value = formatDateTimeLocal(endDate);
        }

        updateSelectionSummary({
            title: machine.name,
            details: `${formatReadableDate(day)} · ${minutesToTime(startMinutes)} – ${minutesToTime(endMinutes)}`
        });
        setFormMessage('', 'info');
        render();
    }

    function selectReservationEvent(reservation, machine) {
        state.selectedSlot = null;
        state.selectedEventId = reservation.id;
        const date = parseDateKey(reservation.date);
        const ownerText = config.userId !== null && reservation.userId === config.userId
            ? 'Votre réservation'
            : `Réservé par ${reservation.userName}`;
        const motifText = reservation.motif ? ` · ${reservation.motif}` : '';

        updateSelectionSummary({
            title: machine.name,
            details: `${formatReadableDate(date)} · ${reservation.start} – ${reservation.end} · ${ownerText}${motifText}`
        });
        render();
    }

    function clearSelection(options = {}) {
        state.selectedSlot = null;
        state.selectedEventId = null;

        if (elements.startInput) {
            elements.startInput.value = '';
        }
        if (elements.endInput) {
            elements.endInput.value = '';
        }
        if (elements.selectionSummary) {
            elements.selectionSummary.hidden = true;
        }
        if (elements.selectionPill) {
            elements.selectionPill.classList.remove('is-selected');
            elements.selectionPill.innerHTML = '<span class="selection-pill__dot" aria-hidden="true"></span>Aucun créneau sélectionné';
        }
        if (!options.silent) {
            setFormMessage('', 'info');
            render();
        }
    }

    function updateSelectionSummary(selection) {
        if (elements.selectionSummary) {
            elements.selectionSummary.hidden = false;
        }
        if (elements.selectionTitle) {
            elements.selectionTitle.textContent = selection.title;
        }
        if (elements.selectionDetails) {
            elements.selectionDetails.textContent = selection.details;
        }
        if (elements.selectionPill) {
            elements.selectionPill.classList.add('is-selected');
            elements.selectionPill.innerHTML = '<span class="selection-pill__dot" aria-hidden="true"></span>Créneau sélectionné';
        }
    }

    async function submitReservation(event) {
        event.preventDefault();

        const machine = getSelectedBookingMachine();
        const startValue = elements.startInput?.value ?? '';
        const endValue = elements.endInput?.value ?? '';

        if (!machine) {
            failForm('Sélectionnez une machine autorisée.');
            return;
        }
        if (!machine.authorized) {
            const missing = machine.missingBadges.length > 0
                ? ` Badge manquant : ${machine.missingBadges.join(', ')}.`
                : '';
            failForm(`Formation nécessaire avant réservation.${missing}`);
            return;
        }
        if (!startValue || !endValue) {
            failForm('Sélectionnez un créneau dans le calendrier ou renseignez les dates.');
            return;
        }

        const startDate = new Date(startValue);
        const endDate = new Date(endValue);
        if (Number.isNaN(startDate.getTime()) || Number.isNaN(endDate.getTime())) {
            failForm('Les dates saisies sont invalides.');
            return;
        }
        if (startDate <= new Date()) {
            failForm('La réservation doit commencer dans le futur.');
            return;
        }
        if (endDate <= startDate) {
            failForm('La fin doit être postérieure au début.');
            return;
        }
        if (formatDateKey(startDate) !== formatDateKey(endDate)) {
            failForm('Une réservation doit rester sur la même journée.');
            return;
        }

        const machineStatus = classifyMachineStatus(machine.status);
        if (machineStatus !== 'available') {
            failForm('Cette machine est actuellement indisponible.');
            return;
        }

        const startMinutes = startDate.getHours() * 60 + startDate.getMinutes();
        const endMinutes = endDate.getHours() * 60 + endDate.getMinutes();
        if (!isPeriodWithinOpening(startDate, startMinutes, endMinutes)) {
            failForm('Le FabLab est fermé sur ce créneau.');
            return;
        }
        if (hasOverlap(machine.id, startDate, endDate)) {
            failForm('Cette machine est déjà réservée sur cette période.');
            return;
        }

        const body = {
            machineId: machine.id,
            dateDebut: startValue,
            dateFin: endValue,
            motif: elements.reasonInput?.value?.trim() ?? ''
        };

        setSubmitting(true);
        setFormMessage('Création de la réservation…', 'info');

        try {
            const response = await fetch(config.reservationUrl, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                    'Accept': 'application/json'
                },
                credentials: 'same-origin',
                body: JSON.stringify(body)
            });

            const responseData = await response.json().catch(() => ({}));
            if (!response.ok) {
                throw new Error(responseData.error || 'La réservation a été refusée par le serveur.');
            }

            setFormMessage('Réservation créée avec succès. Actualisation du calendrier…', 'success');
            showToast('Réservation confirmée', `${machine.name} a bien été réservée.`, 'success');
            window.setTimeout(() => window.location.reload(), 850);
        } catch (error) {
            const message = error instanceof Error ? error.message : 'Une erreur inattendue est survenue.';
            setFormMessage(message, 'error');
            showToast('Réservation impossible', message, 'error');
            setSubmitting(false);
        }
    }

    function failForm(message) {
        setFormMessage(message, 'error');
        showToast('Vérification nécessaire', message, 'error');
    }

    function setSubmitting(isSubmitting) {
        if (!elements.submit) {
            return;
        }
        elements.submit.disabled = isSubmitting;
        elements.submit.setAttribute('aria-busy', isSubmitting ? 'true' : 'false');
    }

    function setFormMessage(message, type) {
        if (!elements.formMessage) {
            return;
        }
        elements.formMessage.textContent = message;
        elements.formMessage.classList.remove('is-error', 'is-success', 'is-info');
        if (message) {
            elements.formMessage.classList.add(`is-${type}`);
        }
    }

    function refreshMachineAvailabilityMessage(machineId) {
        const machine = machineMap.get(machineId);
        if (!machine) {
            return;
        }
        const status = classifyMachineStatus(machine.status);
        if (!machine.authorized) {
            const missing = machine.missingBadges.length > 0
                ? ` Badge manquant : ${machine.missingBadges.join(', ')}.`
                : '';
            setFormMessage(`Formation nécessaire avant réservation.${missing}`, 'error');
        } else if (status === 'maintenance') {
            setFormMessage(`${machine.name} est actuellement en maintenance.`, 'error');
        } else if (status === 'broken') {
            setFormMessage(`${machine.name} est actuellement en panne.`, 'error');
        } else {
            setFormMessage('', 'info');
        }
    }

    function getSelectedBookingMachine() {
        const machineId = Number(elements.machineSelect?.value ?? 0);
        return machineMap.get(machineId) ?? null;
    }

    function ensureMachineVisible(machineId) {
        if (state.selectedMachineIds.has(machineId)) {
            return;
        }
        state.selectedMachineIds.add(machineId);
        const checkbox = elements.filterCheckboxes.find(item => Number(item.dataset.machineId) === machineId);
        if (checkbox) {
            checkbox.checked = true;
        }
        render();
    }

    function getVisibleWeekReservations(weekDays) {
        const dateKeys = new Set(weekDays.map(formatDateKey));
        return reservations.filter(reservation =>
            dateKeys.has(reservation.date) && state.selectedMachineIds.has(reservation.machineId)
        );
    }

    function renderStats(visibleReservations) {
        const selectedCount = state.selectedMachineIds.size;
        setText(elements.selectedMachinesCount, selectedCount);
        setText(elements.visibleMachinesCount, selectedCount);
        setText(elements.weekReservationsCount, visibleReservations.length);
        setText(
            elements.openDaysCount,
            Array.from(openingHours.values()).filter(row => !row.isClosed).length
        );
    }

    function renderEmptyState(visibleReservations) {
        if (elements.emptyState) {
            elements.emptyState.hidden = visibleReservations.length !== 0;
        }
    }

    function assignOverlapLanes(list) {
        const result = [];
        const byDate = new Map();

        list.forEach(reservation => {
            const normalized = {
                ...reservation,
                startMinutes: timeToMinutes(reservation.start),
                endMinutes: timeToMinutes(reservation.end)
            };
            if (!byDate.has(normalized.date)) {
                byDate.set(normalized.date, []);
            }
            byDate.get(normalized.date).push(normalized);
        });

        byDate.forEach(dayReservations => {
            dayReservations.sort((a, b) => a.startMinutes - b.startMinutes || a.endMinutes - b.endMinutes);
            let currentGroup = [];
            let currentGroupEnd = -1;

            const flushGroup = () => {
                if (currentGroup.length === 0) {
                    return;
                }

                const laneEnds = [];
                currentGroup.forEach(reservation => {
                    let laneIndex = laneEnds.findIndex(end => end <= reservation.startMinutes);
                    if (laneIndex === -1) {
                        laneIndex = laneEnds.length;
                        laneEnds.push(reservation.endMinutes);
                    } else {
                        laneEnds[laneIndex] = reservation.endMinutes;
                    }
                    reservation.laneIndex = laneIndex;
                });

                const laneCount = Math.max(1, laneEnds.length);
                currentGroup.forEach(reservation => {
                    reservation.laneCount = laneCount;
                    result.push(reservation);
                });
                currentGroup = [];
                currentGroupEnd = -1;
            };

            dayReservations.forEach(reservation => {
                if (currentGroup.length > 0 && reservation.startMinutes >= currentGroupEnd) {
                    flushGroup();
                }
                currentGroup.push(reservation);
                currentGroupEnd = Math.max(currentGroupEnd, reservation.endMinutes);
            });
            flushGroup();
        });

        return result;
    }

    function hasOverlap(machineId, startDate, endDate) {
        const dateKey = formatDateKey(startDate);
        const startMinutes = startDate.getHours() * 60 + startDate.getMinutes();
        const endMinutes = endDate.getHours() * 60 + endDate.getMinutes();

        return reservations.some(reservation => {
            if (reservation.machineId !== machineId || reservation.date !== dateKey) {
                return false;
            }
            const reservationStart = timeToMinutes(reservation.start);
            const reservationEnd = timeToMinutes(reservation.end);
            return reservationStart < endMinutes && reservationEnd > startMinutes;
        });
    }

    function classifyMachineStatus(status) {
        const normalized = String(status ?? '').trim().toLowerCase();
        if (normalized.includes('maintenance')) {
            return 'maintenance';
        }
        if (
            normalized.includes('panne') ||
            normalized.includes('hors') ||
            normalized.includes('broken') ||
            normalized.includes('offline')
        ) {
            return 'broken';
        }
        return 'available';
    }

    function isPeriodWithinOpening(date, startMinutes, endMinutes) {
        const dayIndex = mondayBasedDayIndex(date);
        const hours = openingHours.get(dayIndex);
        if (!hours || hours.isClosed || hours.openMinutes === null || hours.closeMinutes === null) {
            return false;
        }
        return startMinutes >= hours.openMinutes && endMinutes <= hours.closeMinutes;
    }

    function scrollToRelevantDay(weekDays) {
        if (!elements.scroll || elements.scroll.scrollWidth <= elements.scroll.clientWidth) {
            return;
        }

        const todayIndex = weekDays.findIndex(day => isSameDay(day, new Date()));
        const targetIndex = todayIndex >= 0 ? todayIndex : 0;
        const timeColumnWidth = 74;
        const dayWidth = Math.max(126, (elements.scroll.scrollWidth - timeColumnWidth) / 7);
        const left = Math.max(0, timeColumnWidth + targetIndex * dayWidth - 16);
        elements.scroll.scrollLeft = left;
    }

    function showToast(title, message, type = 'info') {
        if (!elements.toastRegion) {
            return;
        }

        const toast = document.createElement('div');
        toast.className = `calendar-toast is-${type}`;
        toast.setAttribute('role', type === 'error' ? 'alert' : 'status');
        const icon = type === 'success' ? '✓' : type === 'error' ? '!' : 'i';
        toast.innerHTML = `
            <span class="calendar-toast__icon" aria-hidden="true">${icon}</span>
            <span class="calendar-toast__copy">
                <strong>${escapeHtml(title)}</strong>
                <span>${escapeHtml(message)}</span>
            </span>
            <button type="button" class="calendar-toast__close" aria-label="Fermer">×</button>
        `;

        const close = () => toast.remove();
        toast.querySelector('.calendar-toast__close')?.addEventListener('click', close);
        elements.toastRegion.appendChild(toast);
        window.setTimeout(close, 5200);
    }

    function getWeekDays(referenceDate) {
        const monday = getWeekStart(referenceDate);
        return Array.from({ length: 7 }, (_, index) => {
            const day = new Date(monday);
            day.setDate(monday.getDate() + index);
            return day;
        });
    }

    function getWeekStart(date) {
        const result = startOfDay(new Date(date));
        const day = result.getDay();
        const difference = day === 0 ? -6 : 1 - day;
        result.setDate(result.getDate() + difference);
        return result;
    }

    function getISOWeek(date) {
        const target = new Date(Date.UTC(date.getFullYear(), date.getMonth(), date.getDate()));
        const dayNumber = target.getUTCDay() || 7;
        target.setUTCDate(target.getUTCDate() + 4 - dayNumber);
        const yearStart = new Date(Date.UTC(target.getUTCFullYear(), 0, 1));
        return Math.ceil((((target - yearStart) / 86400000) + 1) / 7);
    }

    function mondayBasedDayIndex(date) {
        return (date.getDay() + 6) % 7;
    }

    function timeToMinutes(value) {
        if (!value || typeof value !== 'string') {
            return null;
        }
        const parts = value.split(':').map(Number);
        if (parts.length < 2 || parts.some(Number.isNaN)) {
            return null;
        }
        return parts[0] * 60 + parts[1];
    }

    function minutesToTime(minutes) {
        const normalized = Math.max(0, minutes);
        const hours = Math.floor(normalized / 60) % 24;
        const mins = normalized % 60;
        return `${String(hours).padStart(2, '0')}:${String(mins).padStart(2, '0')}`;
    }

    function dateWithMinutes(day, minutes) {
        const date = new Date(day);
        date.setHours(Math.floor(minutes / 60), minutes % 60, 0, 0);
        return date;
    }

    function roundDateUp(date, stepMinutes) {
        const rounded = new Date(date);
        rounded.setSeconds(0, 0);
        const remainder = rounded.getMinutes() % stepMinutes;
        if (remainder !== 0) {
            rounded.setMinutes(rounded.getMinutes() + stepMinutes - remainder);
        }
        return rounded;
    }

    function startOfDay(date) {
        const result = new Date(date);
        result.setHours(0, 0, 0, 0);
        return result;
    }

    function formatDateKey(date) {
        return [
            date.getFullYear(),
            String(date.getMonth() + 1).padStart(2, '0'),
            String(date.getDate()).padStart(2, '0')
        ].join('-');
    }

    function formatDateTimeLocal(date) {
        return `${formatDateKey(date)}T${String(date.getHours()).padStart(2, '0')}:${String(date.getMinutes()).padStart(2, '0')}`;
    }

    function parseDateKey(value) {
        const [year, month, day] = value.split('-').map(Number);
        return new Date(year, month - 1, day);
    }

    function formatReadableDate(date) {
        return date.toLocaleDateString('fr-FR', {
            weekday: 'long',
            day: 'numeric',
            month: 'long'
        });
    }

    function isSameDay(first, second) {
        return first.getFullYear() === second.getFullYear()
            && first.getMonth() === second.getMonth()
            && first.getDate() === second.getDate();
    }

    function setText(element, value) {
        if (element) {
            element.textContent = String(value);
        }
    }

    function escapeHtml(value) {
        return String(value)
            .replaceAll('&', '&amp;')
            .replaceAll('<', '&lt;')
            .replaceAll('>', '&gt;')
            .replaceAll('"', '&quot;')
            .replaceAll("'", '&#039;');
    }
})();
