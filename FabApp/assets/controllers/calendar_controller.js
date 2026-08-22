import { Controller } from '@hotwired/stimulus';

/**
 * THE calendar. One component, every calendar surface (S146a).
 *
 * 🔴 **Why this file exists, measured rather than felt.** `calendrier.html.twig`
 * (664 lines) and `machine-calendrier.html.twig` (288) each carried their own
 * copy of the same twelve functions — `timeToMinutes`, `formatDateKey`,
 * `getWeekDays`, `isMinuteOpen`, `getSlotState`, `exceptionFor`,
 * `buildTimeOptions`, `overlapsHour`, `reservationCardHtml`,
 * `getOpeningHoursLabel`, `findNextOpenSlotLabel`, `getWeekNumber`. In the
 * single session S134d/S134e the **same** rule had to be edited in both files
 * four times over (several ranges a day, `isMinuteOpen`, `getSlotState`, dated
 * exceptions). Every one of those edits was a chance for the two to disagree
 * about when the lab is open — and the calendar disagreeing with the booking
 * gate is the fault the whole schedule model was rebuilt to prevent.
 *
 * ⚠️ **The server rules are mirrored here, deliberately, and must stay mirrored.**
 * A dated exception answers BEFORE the weekday (S134e) and the RANGES decide
 * bookability, never the day's envelope (S134d) — exactly what
 * `ScheduleResolver::openIntervalsFor()` does. The envelope is layout only.
 *
 * ⚠️ **One rule for a visitor who is not signed in: the grid stays open and the
 * panel invites them to sign in.** The two pages disagreed about this — the
 * machine calendar greyed its whole week out for anonymous visitors, which tells
 * a prospective member the lab is unavailable rather than that they need an
 * account. The booking form is gated server-side by `{% if app.user %}` either
 * way, so nothing is loosened here.
 *
 * ⚠️ **It never decides anything.** Every verdict — who may book what, when a
 * location is open, whose name may be shown — arrives already computed in the
 * payload. Adding a rule to this file instead of to a service is how the two
 * calendars drifted in the first place.
 */
export default class extends Controller {
    static targets = [
        'periodLabel', 'weekHours', 'grid', 'viewButton', 'viewLink',
        'previousButton', 'nextButton',
        'todayHours', 'todayCount', 'visibleCount', 'nextSlot',
        'resourceToggle', 'resourceSearch', 'resourceStatus', 'resourceCard',
        'panel', 'backdrop', 'form', 'startInput', 'endInput',
        'dateLabel', 'timeLabel', 'context', 'startTime', 'endTime',
        'resourceSelect', 'durationChip', 'submit', 'message',
    ];

    static values = { payload: Object };

    connect() {
        const p = this.payloadValue;
        this.p = p;
        this.hoursByDay = {};
        this.rangesByDay = {};
        (p.hours || []).forEach((row) => {
            this.hoursByDay[row.dayIndex] = row.isClosed
                ? null
                : [this.toMinutes(row.openTime), this.toMinutes(row.closeTime)];
            this.rangesByDay[row.dayIndex] = (row.ranges || []).map((r) => [this.toMinutes(r[0]), this.toMinutes(r[1])]);
        });
        this.exceptions = p.exceptions || {};
        this.access = p.access || {};
        this.resources = p.resources || [];
        this.reservations = p.reservations || [];
        this.events = p.events || [];
        this.labels = p.labels || {};
        this.stepMinutes = 15;
        this.selectedKeys = new Set(this.resources.map((r) => r.key));
        this.currentDate = new Date();
        this.draft = null;
        this.view = this.initialView();
        this.escapeHandler = (event) => {
            if (event.key === 'Escape' && this.hasPanelTarget && this.panelTarget.classList.contains('is-open')) {
                this.closePanel();
            }
        };
        document.addEventListener('keydown', this.escapeHandler);
        this.render();
    }

    disconnect() {
        document.removeEventListener('keydown', this.escapeHandler);
    }

    /* ---------------------------------------------------------------- period */

    initialView() {
        const asked = new URLSearchParams(window.location.search).get('view');
        return asked === 'month' ? 'month' : 'week';
    }

    previous() {
        this.shift(-1);
    }

    next() {
        this.shift(1);
    }

    shift(direction) {
        if (this.view === 'month') {
            this.currentDate = new Date(this.currentDate.getFullYear(), this.currentDate.getMonth() + direction, 1);
        } else {
            this.currentDate.setDate(this.currentDate.getDate() + (7 * direction));
        }
        this.render();
    }

    today() {
        this.currentDate = new Date();
        this.render();
    }

    /**
     * ⚠️ The view is written back into the URL rather than kept in a variable.
     * The location filter is a server-rendered link, so a month the browser knew
     * about but the address bar did not would be lost the moment somebody
     * switched location — and could not be sent to a colleague either.
     */
    setView(event) {
        event.preventDefault();
        const asked = event.currentTarget.dataset.calendarView === 'month' ? 'month' : 'week';
        if (asked === this.view) {
            return;
        }
        this.view = asked;
        const url = new URL(window.location.href);
        if (asked === 'week') {
            url.searchParams.delete('view');
        } else {
            url.searchParams.set('view', asked);
        }
        window.history.replaceState({}, '', url);
        this.render();
    }

    /* ---------------------------------------------------------------- render */

    render() {
        this.element.querySelectorAll('[data-calendar-week-only]').forEach((chrome) => {
            chrome.hidden = this.view === 'month';
        });
        this.renderViewSwitch();
        this.renderPeriodLabel();
        this.renderStats();
        this.renderWeekHours();
        if (this.view === 'month') {
            this.renderMonth();
        } else {
            this.renderWeek();
        }
    }

    renderViewSwitch() {
        // ⚠️ The step buttons say what they actually do. "Semaine précédente" on a
        // month grid is a name that lies about the control it is on — and since the
        // buttons are arrows, that name is all a screen reader or a tooltip has.
        this.nameStep(this.hasPreviousButtonTarget ? this.previousButtonTarget : null, this.view === 'month' ? this.labels.navPrevMonth : this.labels.navPrevWeek);
        this.nameStep(this.hasNextButtonTarget ? this.nextButtonTarget : null, this.view === 'month' ? this.labels.navNextMonth : this.labels.navNextWeek);
        this.viewButtonTargets.forEach((button) => {
            const on = button.dataset.calendarView === this.view;
            button.classList.toggle('is-on', on);
            button.setAttribute('aria-pressed', on ? 'true' : 'false');
        });
        // Every server-rendered link out of this component (the location tiles)
        // carries the current view, so changing location does not silently drop it.
        this.viewLinkTargets.forEach((link) => {
            const url = new URL(link.href, window.location.origin);
            if (this.view === 'week') {
                url.searchParams.delete('view');
            } else {
                url.searchParams.set('view', this.view);
            }
            link.href = url.pathname + url.search;
        });
    }

    nameStep(button, label) {
        if (!button) {
            return;
        }
        button.title = label;
        button.setAttribute('aria-label', label);
    }

    renderPeriodLabel() {
        if (!this.hasPeriodLabelTarget) {
            return;
        }
        if (this.view === 'month') {
            this.periodLabelTarget.textContent = this.currentDate.toLocaleDateString(window.FABOS_LOCALE, { month: 'long', year: 'numeric' });
            return;
        }
        const start = this.weekStart(this.currentDate);
        const end = new Date(start);
        end.setDate(end.getDate() + 6);
        this.periodLabelTarget.textContent = `${this.labels.week} ${this.weekNumber(this.currentDate)} · `
            + `${start.toLocaleDateString(window.FABOS_LOCALE, { day: 'numeric', month: 'short' })} - `
            + `${end.toLocaleDateString(window.FABOS_LOCALE, { day: 'numeric', month: 'short', year: 'numeric' })}`;
    }

    renderStats() {
        const today = new Date();
        const todayIndex = today.getDay() === 0 ? 6 : today.getDay() - 1;
        const todayKey = this.dateKey(today);
        if (this.hasTodayHoursTarget) {
            this.todayHoursTarget.textContent = this.openingLabel(todayIndex, today);
        }
        if (this.hasTodayCountTarget) {
            this.todayCountTarget.textContent = this.visibleReservations().filter((res) => res.date === todayKey).length;
        }
        if (this.hasVisibleCountTarget) {
            this.visibleCountTarget.textContent = this.visibleResources().length;
        }
        if (this.hasNextSlotTarget) {
            this.nextSlotTarget.textContent = this.nextOpenSlotLabel();
        }
    }

    renderWeekHours() {
        if (!this.hasWeekHoursTarget) {
            return;
        }
        const html = this.weekDays().map((day, index) => {
            const closed = this.hoursByDay[index] === null || this.hoursByDay[index] === undefined;
            return `<span class="week-hour-chip${closed ? ' closed' : ''}">`
                + `<strong>${this.escape(day.toLocaleDateString(window.FABOS_LOCALE, { weekday: 'short', day: 'numeric' }))}</strong>`
                + `<br>${this.escape(this.openingLabel(index, day))}</span>`;
        }).join('');
        this.weekHoursTargets.forEach((strip) => { strip.innerHTML = html; });
    }

    renderWeek() {
        const grid = this.gridTarget;
        grid.classList.remove('is-month');
        const days = this.weekDays();
        const hours = this.timeSlots();
        const reservations = this.visibleReservations();
        const resources = this.visibleResources();
        const namesShown = this.resources.length > 1;

        let html = `<div class="agenda-header"><div class="agenda-time-head">${this.escape(this.labels.hour)}</div>`;
        html += days.map((day) => `<div class="agenda-day-head${this.sameDay(day, new Date()) ? ' today' : ''}">`
            + `<span>${this.escape(day.toLocaleDateString(window.FABOS_LOCALE, { weekday: 'short', day: 'numeric' }))}</span>`
            + `<small>${this.escape(this.openingLabel(day.getDay() === 0 ? 6 : day.getDay() - 1, day))}</small></div>`).join('');
        html += '</div>';

        hours.forEach((hour) => {
            html += `<div class="agenda-row"><div class="agenda-time-cell">${hour}:00</div>`;
            days.forEach((day, dayIndex) => {
                const key = this.dateKey(day);
                const slotStart = parseInt(hour, 10) * 60;
                const slotReservations = reservations.filter((res) => res.date === key && this.overlapsHour(res, slotStart));
                const slotEvents = this.events.filter((ev) => ev.date === key && ev.hour === hour);
                const state = this.slotState(dayIndex, slotStart, day);
                // ⚠️ The slot is one hour; a package window has to cover all of it.
                const bookable = resources.some((r) => this.canReserveAt(r.key, dayIndex, slotStart, slotStart + 60));
                const outsideWindows = this.p.authenticated
                    && !bookable
                    && resources.some((r) => this.canReserve(r.key));
                const locked = this.p.authenticated && !bookable;
                const canBook = this.p.booking && !state.closed && !locked;
                const lockLabel = outsideWindows ? this.labels.outsidePackageHours : this.lockLabelFor(resources);
                const className = `${state.className}${locked ? ' is-training-locked' : ''}${slotEvents.length ? ' has-event' : ''}`;
                const title = locked ? lockLabel : this.labels.clickToBook;
                html += `<div class="agenda-slot-cell ${className}" data-day="${dayIndex}" data-hour="${hour}"`
                    + (canBook ? ` role="button" tabindex="0" title="${this.escape(title)}" aria-label="${this.escape(title)}"` : '')
                    + '>';
                slotEvents.forEach((ev) => { html += this.eventCard(ev); });
                if (state.closed) {
                    html += `<span class="slot-state-label">${this.escape(state.label)}</span>`;
                } else {
                    slotReservations.slice(0, 3).forEach((res) => { html += this.reservationCard(res, namesShown); });
                    if (slotReservations.length > 3) {
                        html += `<button type="button" class="slot-more-button">${this.escape(String(this.labels.moreBookings).replace('%count%', slotReservations.length - 3))}</button>`;
                    }
                    if (locked) {
                        html += `<span class="slot-state-label training-lock-label">${this.escape(lockLabel)}</span>`;
                    } else if (this.p.booking) {
                        html += `<span class="slot-book-affordance" aria-hidden="true">${this.p.unavailable ? '!' : '+'}</span>`;
                    }
                }
                html += '</div>';
            });
            html += '</div>';
        });

        grid.innerHTML = html;
        grid.querySelectorAll('.agenda-slot-cell.is-open[role="button"]').forEach((cell) => {
            const open = () => this.openPanel(parseInt(cell.dataset.day, 10), cell.dataset.hour);
            cell.addEventListener('click', (event) => {
                if (event.target.closest('.slot-reservation-card, .slot-more-button, .slot-event-card')) {
                    return;
                }
                open();
            });
            cell.addEventListener('keydown', (event) => {
                if (event.key !== 'Enter' && event.key !== ' ') {
                    return;
                }
                event.preventDefault();
                open();
            });
        });
    }

    /**
     * The month. Not a second calendar: it reads the same schedule, the same
     * bookings and the same events as the week, and answers the question the week
     * cannot — "when, over the next few weeks, is this thing free at all?"
     *
     * ⚠️ A day is a **summary**, not a booking surface. Booking needs an hour, and
     * inventing one from a month cell is how a member ends up with a slot they did
     * not choose; clicking a day opens that day's week instead.
     */
    renderMonth() {
        const grid = this.gridTarget;
        grid.classList.add('is-month');
        const first = new Date(this.currentDate.getFullYear(), this.currentDate.getMonth(), 1);
        const start = this.weekStart(first);
        const month = this.currentDate.getMonth();
        const reservations = this.visibleReservations();
        const today = new Date();

        let html = '<div class="month-header">';
        for (let i = 0; i < 7; i += 1) {
            const day = new Date(start);
            day.setDate(start.getDate() + i);
            html += `<div class="month-day-head">${this.escape(day.toLocaleDateString(window.FABOS_LOCALE, { weekday: 'short' }))}</div>`;
        }
        html += '</div>';

        for (let week = 0; week < 6; week += 1) {
            html += '<div class="month-row">';
            for (let index = 0; index < 7; index += 1) {
                const day = new Date(start);
                day.setDate(start.getDate() + (week * 7) + index);
                const dayIndex = day.getDay() === 0 ? 6 : day.getDay() - 1;
                const key = this.dateKey(day);
                const exception = this.exceptionFor(day);
                const closed = (exception && exception.closed)
                    || (!exception && (this.hoursByDay[dayIndex] === null || this.hoursByDay[dayIndex] === undefined));
                const count = reservations.filter((res) => res.date === key).length;
                const dayEvents = this.events.filter((ev) => ev.date === key);
                const classes = ['month-day'];
                if (day.getMonth() !== month) classes.push('is-outside');
                if (closed) classes.push('is-closed');
                if (this.sameDay(day, today)) classes.push('today');
                const reason = closed ? ((exception && exception.reason) || this.labels.venueClosed) : this.openingLabel(dayIndex, day);
                html += `<div class="${classes.join(' ')}" data-date="${key}" role="button" tabindex="0"`
                    + ` title="${this.escape(this.labels.openWeek)}" aria-label="${this.escape(`${day.toLocaleDateString(window.FABOS_LOCALE, { day: 'numeric', month: 'long' })} · ${reason}`)}">`
                    + `<span class="month-day-number">${day.getDate()}</span>`
                    + `<span class="month-day-hours">${this.escape(reason)}</span>`;
                dayEvents.slice(0, 2).forEach((ev) => {
                    html += `<span class="month-day-event" title="${this.escape(ev.category ? `${ev.category} · ${ev.title}` : ev.title)}">`
                        + `${this.escape(ev.startLabel)} · ${this.escape(ev.title)}</span>`;
                });
                if (dayEvents.length > 2) {
                    html += `<span class="month-day-more">${this.escape(String(this.labels.moreBookings).replace('%count%', dayEvents.length - 2))}</span>`;
                }
                if (count > 0) {
                    html += `<span class="month-day-count">${this.escape(String(this.labels.bookingsCount).replace('%count%', count))}</span>`;
                }
                html += '</div>';
            }
            html += '</div>';
        }

        grid.innerHTML = html;
        grid.querySelectorAll('.month-day').forEach((cell) => {
            const open = () => {
                const [year, m, d] = cell.dataset.date.split('-').map(Number);
                this.currentDate = new Date(year, m - 1, d);
                this.view = 'week';
                const url = new URL(window.location.href);
                url.searchParams.delete('view');
                window.history.replaceState({}, '', url);
                this.render();
            };
            cell.addEventListener('click', open);
            cell.addEventListener('keydown', (event) => {
                if (event.key !== 'Enter' && event.key !== ' ') {
                    return;
                }
                event.preventDefault();
                open();
            });
        });
    }

    /**
     * ⚠️ `mine` is decided by the SERVER and `res.user` is null when the viewer may
     * not see who booked — the slot must still read as taken. Only your own slot is
     * a link, because `/reservations/{id}` answers 404 to anyone else: a link on
     * someone else's booking would be a control that cannot work, and it would put
     * their booking id on the wire (S38).
     */
    reservationCard(res, withName) {
        const mine = res.mine;
        const status = res.statut === 'cancelled' ? this.labels.cancelled : this.labels.confirmed;
        const who = mine ? this.labels.mine : (res.user ? this.escape(res.user) : this.labels.taken);
        const heading = withName && res.machine
            ? `${this.escape(res.start)} - ${this.escape(res.end)} · ${this.escape(res.machine)}`
            : `${this.escape(res.start)} - ${this.escape(res.end)}`;
        const body = `<strong>${heading}</strong><span>${who} · ${this.escape(status)}</span>`
            + (res.motif ? `<span>${this.escape(res.motif)}</span>` : '');
        const cls = `slot-reservation-card${mine ? ' mine' : ''}${res.statut === 'cancelled' ? ' cancelled' : ''}`;
        if (!res.url) {
            return `<article class="${cls}">${body}</article>`;
        }
        return `<a class="${cls} is-linked" href="${this.escape(res.url)}" title="${this.escape(this.labels.manage)}">${body}</a>`;
    }

    // ⚠️ `ev.category` is the lab's own word and arrives already chosen by the
    // operator — never translated, never branched on (see `EventCategory`).
    eventCard(ev) {
        return `<a class="slot-event-card" href="${this.escape(ev.url)}" title="${this.escape(ev.title)}">`
            + `<strong>${this.escape(ev.startLabel)} · ${this.escape(ev.title)}</strong>`
            + (ev.category ? `<span class="slot-event-kind">${this.escape(ev.category)}</span>` : '')
            + (ev.lieu ? `<span>${this.escape(ev.lieu)}</span>` : '')
            + '</a>';
    }

    /* --------------------------------------------------------- resource list */

    filterResources() {
        const query = this.hasResourceSearchTarget ? this.resourceSearchTarget.value.trim().toLowerCase() : '';
        const status = this.hasResourceStatusTarget ? this.resourceStatusTarget.value : 'all';
        this.resourceCardTargets.forEach((card) => {
            const matchesQuery = !query || `${card.dataset.name} ${card.dataset.status} ${card.dataset.category}`.includes(query);
            const matchesStatus = status === 'all' || card.dataset.status === status;
            card.style.display = matchesQuery && matchesStatus ? '' : 'none';
        });
    }

    resetResources() {
        if (this.hasResourceSearchTarget) this.resourceSearchTarget.value = '';
        if (this.hasResourceStatusTarget) this.resourceStatusTarget.value = 'all';
        this.resourceToggleTargets.forEach((input) => {
            input.checked = true;
            this.selectedKeys.add(input.value);
        });
        this.filterResources();
        this.render();
    }

    toggleResource(event) {
        const input = event.currentTarget;
        if (input.checked) {
            this.selectedKeys.add(input.value);
        } else {
            this.selectedKeys.delete(input.value);
        }
        this.render();
    }

    visibleResources() {
        return this.resources.filter((resource) => this.selectedKeys.has(resource.key));
    }

    visibleReservations() {
        return this.reservations.filter((res) => res.resource_key
            && this.selectedKeys.has(res.resource_key)
            && res.statut !== 'cancelled');
    }

    /* -------------------------------------------------------- booking panel */

    openPanel(dayIndex, hourStr) {
        if (!this.hasPanelTarget) {
            return;
        }
        this.panelTarget.classList.add('is-open');
        this.panelTarget.setAttribute('aria-hidden', 'false');
        if (this.hasBackdropTarget) this.backdropTarget.classList.add('is-open');
        document.body.classList.add('booking-modal-open');

        if (!this.p.authenticated) {
            const start = this.dateTimeLocal(dayIndex, hourStr, 0);
            const end = this.dateTimeLocal(dayIndex, hourStr, this.p.slotMinutes);
            if (this.hasContextTarget) {
                this.contextTarget.textContent = String(this.labels.slotContext)
                    .replace('%machine%', this.resources.length ? this.resources[0].name : '')
                    .replace('%start%', start.replace('T', ' '))
                    .replace('%end%', end.split('T')[1]);
            }
            return;
        }

        this.setSelection(dayIndex, hourStr);
        if (this.hasResourceSelectTarget) {
            const first = this.visibleResources().find((resource) => this.canReserve(resource.key));
            this.resourceSelectTarget.value = first ? first.key : '';
        }
        if (this.hasSubmitTarget) this.submitTarget.disabled = false;
        this.setMessage('', '');
        this.validate();
        window.setTimeout(() => { if (this.hasStartTimeTarget) this.startTimeTarget.focus(); }, 80);
    }

    closePanel() {
        if (!this.hasPanelTarget) {
            return;
        }
        this.panelTarget.classList.remove('is-open');
        this.panelTarget.setAttribute('aria-hidden', 'true');
        if (this.hasBackdropTarget) this.backdropTarget.classList.remove('is-open');
        document.body.classList.remove('booking-modal-open');
    }

    setSelection(dayIndex, hourStr) {
        const day = this.weekDays()[dayIndex];
        const slotStart = parseInt(hourStr, 10) * 60;
        const opening = this.hoursByDay[dayIndex];
        const end = opening ? Math.min(slotStart + this.p.slotMinutes, opening[1]) : slotStart + this.p.slotMinutes;
        this.draft = { dayIndex, dateKey: this.dateKey(day), date: day, startMinutes: slotStart, endMinutes: end };
        this.populateTimeControls();
        this.syncHiddenFields();
    }

    populateTimeControls() {
        if (!this.draft || !this.hasStartTimeTarget || !this.hasEndTimeTarget) {
            return;
        }
        const opening = this.hoursByDay[this.draft.dayIndex];
        const min = opening ? opening[0] : this.p.startHour * 60;
        const max = opening ? opening[1] : this.p.endHour * 60;
        this.startTimeTarget.innerHTML = this.timeOptions(min, Math.max(min, max - this.stepMinutes), this.draft.startMinutes);
        this.endTimeTarget.innerHTML = this.timeOptions(min + this.stepMinutes, max, this.draft.endMinutes);
        this.startTimeTarget.value = String(this.draft.startMinutes);
        this.endTimeTarget.value = String(this.draft.endMinutes);
    }

    changeStart() {
        this.updateFromControls('start');
    }

    changeEnd() {
        this.updateFromControls('end');
    }

    updateFromControls(changed) {
        if (!this.draft) {
            return;
        }
        this.draft.startMinutes = parseInt(this.startTimeTarget.value, 10);
        this.draft.endMinutes = parseInt(this.endTimeTarget.value, 10);
        if (changed === 'start' && this.draft.endMinutes <= this.draft.startMinutes) {
            this.draft.endMinutes = this.draft.startMinutes + this.p.slotMinutes;
            this.populateTimeControls();
        }
        this.syncHiddenFields();
    }

    duration(event) {
        if (!this.draft) {
            return;
        }
        const minutes = parseInt(event.currentTarget.dataset.calendarDuration, 10);
        const opening = this.hoursByDay[this.draft.dayIndex];
        this.draft.endMinutes = opening
            ? Math.min(this.draft.startMinutes + minutes, opening[1])
            : this.draft.startMinutes + minutes;
        this.durationChipTargets.forEach((chip) => {
            chip.classList.toggle('is-active', parseInt(chip.dataset.calendarDuration, 10) === minutes);
        });
        this.populateTimeControls();
        this.syncHiddenFields();
    }

    syncHiddenFields() {
        if (!this.draft) {
            return;
        }
        this.startInputTarget.value = `${this.draft.dateKey}T${this.clock(this.draft.startMinutes)}`;
        this.endInputTarget.value = `${this.draft.dateKey}T${this.clock(this.draft.endMinutes)}`;
        if (this.hasDateLabelTarget) {
            this.dateLabelTarget.textContent = this.draft.date.toLocaleDateString(window.FABOS_LOCALE, { weekday: 'long', day: 'numeric', month: 'long', year: 'numeric' });
        }
        if (this.hasTimeLabelTarget) {
            this.timeLabelTarget.textContent = `${this.clock(this.draft.startMinutes)} - ${this.clock(this.draft.endMinutes)}`;
        }
        if (this.hasContextTarget) {
            this.contextTarget.textContent = String(this.labels.slotDay)
                .replace('%machine%', this.resources.length ? this.resources[0].name : '')
                .replace('%day%', this.draft.date.toLocaleDateString(window.FABOS_LOCALE, { weekday: 'short', day: 'numeric', month: 'short' }));
        }
        this.validate();
    }

    validate() {
        if (!this.draft) {
            return false;
        }
        const opening = this.hoursByDay[this.draft.dayIndex];
        const key = this.selectedResourceKey();
        let message = '';
        if (this.draft.endMinutes <= this.draft.startMinutes) message = this.labels.endBeforeStart;
        if (opening && (this.draft.startMinutes < opening[0] || this.draft.endMinutes > opening[1])) message = this.labels.outsideHours;
        if (this.p.authenticated && key && !this.canReserve(key)) message = this.lockLabelFor([{ key }]);
        if (this.p.authenticated && !key) message = this.labels.noResource;
        const valid = !message;
        if (this.hasSubmitTarget) this.submitTarget.disabled = !valid;
        if (message) {
            this.setMessage(message, 'error');
        } else {
            this.setMessage(this.p.unavailable ? this.labels.unavailableWarning : '', this.p.unavailable ? 'error' : '');
        }
        return valid;
    }

    selectedResourceKey() {
        if (this.hasResourceSelectTarget) {
            return this.resourceSelectTarget.value;
        }
        return this.resources.length ? this.resources[0].key : '';
    }

    async submitBooking(event) {
        event.preventDefault();
        const form = event.currentTarget;
        const key = this.selectedResourceKey();
        if (!this.startInputTarget.value || !this.endInputTarget.value || !key) {
            this.setMessage(this.labels.requiredFields, 'error');
            return;
        }
        if (!this.validate()) {
            return;
        }
        if (new Date(this.endInputTarget.value) <= new Date(this.startInputTarget.value)) {
            this.setMessage(this.labels.endDateBeforeStart, 'error');
            return;
        }
        this.setMessage(this.labels.creating, 'loading');
        if (this.hasSubmitTarget) this.submitTarget.disabled = true;
        try {
            const response = await fetch(this.p.createUrl, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json', Accept: 'application/json' },
                credentials: 'same-origin',
                body: JSON.stringify({
                    reservableType: key.split(':')[0],
                    reservableId: key.split(':')[1],
                    dateDebut: this.startInputTarget.value,
                    dateFin: this.endInputTarget.value,
                    motif: form.motif ? form.motif.value : '',
                }),
            });
            const data = await response.json().catch(() => ({}));
            if (!response.ok) {
                throw new Error(data.error || this.labels.refused);
            }
            this.setMessage(this.labels.created, 'success');
            window.setTimeout(() => window.location.reload(), 700);
        } catch (error) {
            this.setMessage(error.message, 'error');
            if (this.hasSubmitTarget) this.submitTarget.disabled = false;
        }
    }

    setMessage(text, state) {
        if (!this.hasMessageTarget) {
            return;
        }
        this.messageTarget.textContent = text;
        this.messageTarget.className = `booking-message ${state}`;
    }

    /* ------------------------------------------------------------- verdicts */

    canReserve(key) {
        if (!this.p.authenticated) {
            return true;
        }
        if (this.p.admin) {
            return true;
        }
        return Boolean(this.access[key] && this.access[key].canReserve);
    }

    /**
     * 🔴 **S147, J-20 — the mirror of `GrantWindowSet::covers`, and it must stay one.**
     * A package can be sold as "the 3D printers, Thursday 14:00–18:00"
     * (`USAGE_GRANT_WINDOW`). The server enforces that at booking time; before
     * this, the grid knew nothing about it and drew the whole week as bookable, so
     * the member filled the panel on a Monday and was refused at the end.
     *
     * The rule is COVERAGE, not overlap: every minute of the slot has to fall
     * inside the union of that weekday's windows. An overlap test would open a
     * Thursday evening on the strength of a Thursday afternoon, which is exactly
     * the mistake the server class exists to refuse — so if one side of this pair
     * ever changes, change the other in the same commit.
     *
     * ⚠️ **No windows means no restriction**, never "nothing allowed": an admin, a
     * visitor, a disabled feature and a package without windows all arrive here
     * with an empty list, and the week must look exactly as it did before.
     */
    windowsAllow(key, dayIndex, startMinute, endMinute) {
        const windows = (this.access[key] || {}).windows || [];
        if (!windows.length) return true;
        if (endMinute <= startMinute) return true;

        // `dayIndex` is 0 = Monday here; `dayOfWeek` is PHP's `N`, 1 = Monday.
        const isoDay = dayIndex + 1;
        const sameDay = windows
            .filter((w) => Number(w.dayOfWeek) === isoDay)
            .sort((a, b) => a.startMinute - b.startMinute);
        if (!sameDay.length) return false;

        let reached = startMinute;
        for (const w of sameDay) {
            if (w.startMinute > reached) break;   // a gap the sorted rest cannot close
            reached = Math.max(reached, w.endMinute);
            if (reached >= endMinute) return true;
        }
        return reached >= endMinute;
    }

    /** Can this person book THIS resource at THIS moment of the week? */
    canReserveAt(key, dayIndex, startMinute, endMinute) {
        return this.canReserve(key) && this.windowsAllow(key, dayIndex, startMinute, endMinute);
    }

    lockLabelFor(resources) {
        if (resources.length === 1) {
            const access = this.access[resources[0].key] || {};
            return access.reasonLabel || this.labels.practicalLocked;
        }
        const physical = resources.some((resource) => (this.access[resource.key] || {}).reason === 'physical_training_required');
        return physical ? this.labels.practicalLocked : this.labels.trainingRequired;
    }

    /**
     * ⚠️ **A dated exception answers first, and it carries its reason** (S134e):
     * "closed" without a why is a question, not an answer. Then the RANGES decide,
     * never the envelope (S134d) — between two ranges the lab is shut, and a slot
     * drawn as bookable there is an invitation the server refuses.
     */
    slotState(dayIndex, slotStart, date) {
        const exception = date ? this.exceptionFor(date) : null;
        if (exception && exception.closed) {
            return { closed: true, className: 'is-closed', label: exception.reason || this.labels.venueClosed };
        }
        const hours = this.hoursByDay[dayIndex];
        if (!exception && (hours === null || hours === undefined)) {
            return { closed: true, className: 'is-closed', label: this.labels.venueClosed };
        }
        if (!this.isMinuteOpen(dayIndex, slotStart, date)) {
            return { closed: true, className: 'is-out', label: this.labels.outOfHours };
        }
        return { closed: false, className: 'is-open', label: this.labels.available };
    }

    exceptionFor(date) {
        return this.exceptions[this.dateKey(date)] || null;
    }

    isMinuteOpen(dayIndex, minute, date) {
        const exception = date ? this.exceptionFor(date) : null;
        if (exception) {
            if (exception.closed) {
                return false;
            }
            return exception.ranges.some(([from, to]) => minute >= this.toMinutes(from) && minute < this.toMinutes(to));
        }
        const ranges = this.rangesByDay[dayIndex] || [];
        if (!ranges.length) {
            return this.hoursByDay[dayIndex] !== null && this.hoursByDay[dayIndex] !== undefined;
        }
        return ranges.some(([from, to]) => minute >= from && minute < to);
    }

    openingLabel(dayIndex, date) {
        const exception = date ? this.exceptionFor(date) : null;
        if (exception) {
            if (exception.closed) {
                return exception.reason || this.labels.closedShort;
            }
            return exception.ranges.map(([from, to]) => `${from} - ${to}`).join(' · ');
        }
        const ranges = this.rangesByDay[dayIndex] || [];
        if (ranges.length) {
            return ranges.map(([from, to]) => `${this.clock(from)} - ${this.clock(to)}`).join(' · ');
        }
        const hours = this.hoursByDay[dayIndex];
        return hours ? `${this.clock(hours[0])} - ${this.clock(hours[1])}` : this.labels.closedShort;
    }

    nextOpenSlotLabel() {
        const days = this.weekDays();
        const now = new Date();
        for (let d = 0; d < days.length; d += 1) {
            const hours = this.hoursByDay[d];
            if (!hours) {
                continue;
            }
            for (let minutes = hours[0]; minutes < hours[1]; minutes += 60) {
                if (!this.isMinuteOpen(d, minutes, days[d])) {
                    continue;
                }
                const candidate = new Date(days[d]);
                candidate.setHours(Math.floor(minutes / 60), minutes % 60, 0, 0);
                if (candidate > now) {
                    return `${candidate.toLocaleDateString(window.FABOS_LOCALE, { weekday: 'short', day: 'numeric' })} ${this.clock(minutes)}`;
                }
            }
        }
        return this.labels.nextWeek;
    }

    overlapsHour(reservation, slotStart) {
        return this.toMinutes(reservation.start) < slotStart + 60 && this.toMinutes(reservation.end) > slotStart;
    }

    /* ----------------------------------------------------------- primitives */

    toMinutes(value) {
        if (!value) {
            return 0;
        }
        const [h, m] = String(value).split(':').map(Number);
        return (h * 60) + (m || 0);
    }

    clock(total) {
        return `${String(Math.floor(total / 60)).padStart(2, '0')}:${String(total % 60).padStart(2, '0')}`;
    }

    timeOptions(min, max, selected) {
        let html = '';
        for (let minutes = min; minutes <= max; minutes += this.stepMinutes) {
            html += `<option value="${minutes}"${minutes === selected ? ' selected' : ''}>${this.clock(minutes)}</option>`;
        }
        return html;
    }

    weekStart(date) {
        const d = new Date(date);
        const day = d.getDay();
        d.setDate(d.getDate() - day + (day === 0 ? -6 : 1));
        d.setHours(0, 0, 0, 0);
        return d;
    }

    weekDays() {
        const start = this.weekStart(this.currentDate);
        return Array.from({ length: 7 }, (unused, i) => {
            const day = new Date(start);
            day.setDate(start.getDate() + i);
            return day;
        });
    }

    timeSlots() {
        return Array.from(
            { length: this.p.endHour - this.p.startHour },
            (unused, i) => String(this.p.startHour + i).padStart(2, '0'),
        );
    }

    weekNumber(date) {
        const d = new Date(Date.UTC(date.getFullYear(), date.getMonth(), date.getDate()));
        d.setUTCDate(d.getUTCDate() + 4 - (d.getUTCDay() || 7));
        const yearStart = new Date(Date.UTC(d.getUTCFullYear(), 0, 1));
        return Math.ceil((((d - yearStart) / 86400000) + 1) / 7);
    }

    sameDay(a, b) {
        return a.getFullYear() === b.getFullYear() && a.getMonth() === b.getMonth() && a.getDate() === b.getDate();
    }

    dateKey(date) {
        return `${date.getFullYear()}-${String(date.getMonth() + 1).padStart(2, '0')}-${String(date.getDate()).padStart(2, '0')}`;
    }

    dateTimeLocal(dayIndex, hourStr, offsetMinutes) {
        const date = new Date(this.weekDays()[dayIndex]);
        date.setHours(parseInt(hourStr, 10), offsetMinutes, 0, 0);
        return `${this.dateKey(date)}T${String(date.getHours()).padStart(2, '0')}:${String(date.getMinutes()).padStart(2, '0')}`;
    }

    escape(value) {
        return String(value === null || value === undefined ? '' : value)
            .replace(/&/g, '&amp;')
            .replace(/</g, '&lt;')
            .replace(/>/g, '&gt;')
            .replace(/"/g, '&quot;')
            .replace(/'/g, '&#039;');
    }
}
