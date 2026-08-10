<?php

namespace App\Form;

use App\Entity\Venue;
use App\Repository\VenueRepository;
use Symfony\Bridge\Doctrine\Form\Type\EntityType;
use Symfony\Component\Form\AbstractType;
use Symfony\Component\OptionsResolver\OptionsResolver;
use Symfony\Contracts\Translation\TranslatorInterface;

/**
 * The sub-venue picker, for every form that stores one.
 *
 * **Why it exists (S133).** `Machine`, `Place` and `LoanableItem` have carried a
 * NOT NULL `venueId` since S107 and `Event` a nullable one, the lists filter by
 * sub-venue, and S129 made a second sub-venue creatable — but **no create or edit
 * form ever offered the field**. Everything therefore landed on the default venue
 * forever, and a second sub-venue could hold nothing at all. That gap is why
 * Phase G's own exit criterion ("the operator can administer a sub-venue from the
 * canonical interface") was not met.
 *
 * ⚠️ **Archived venues stay in the list, disabled — they are not filtered out.**
 * `VenueGuard` deliberately permits archiving a venue that still holds machines
 * (it states the count and asks). So an existing record can legitimately point at
 * an archived venue, and a picker offering only active ones would make that record
 * **uneditable**: its current value would not be among the choices and every save
 * would fail validation on a field the operator never touched. Listing all venues
 * and disabling the archived ones lets the current value round-trip while still
 * refusing to file anything new into an archive.
 */
final class VenueChoiceType extends AbstractType
{
    public function __construct(private readonly TranslatorInterface $translator)
    {
    }

    public function getParent(): string
    {
        return EntityType::class;
    }

    public function configureOptions(OptionsResolver $resolver): void
    {
        // ⚠️ Translated HERE, not returned as a key. A `choice_label` closure's
        // return value is looked up as a whole, so concatenating a key into it
        // puts `venues.status.archived` on screen verbatim — the same fault that
        // shipped `workspace.tab.app_admin_venues` as a visible tab label.
        $archived = $this->translator->trans('venues.status.archived');

        $resolver->setDefaults([
            'class' => Venue::class,
            'label' => 'venues.field.venue',
            'help' => 'venues.help.venue',
            'query_builder' => static fn (VenueRepository $venues) => $venues
                ->createQueryBuilder('v')
                ->orderBy('v.active', 'DESC')
                ->addOrderBy('v.name', 'ASC'),
            // The suffix is what makes the disabled row explicable rather than
            // mysteriously unselectable.
            'choice_label' => static fn (Venue $venue): string => $venue->isActive()
                ? $venue->getName()
                : $venue->getName() . ' — ' . $archived,
            'choice_attr' => static fn (Venue $venue): array => $venue->isActive()
                ? []
                : ['disabled' => 'disabled'],
            // The composed label is already a finished sentence; letting the form
            // translate it again would look the whole string up and miss.
            'choice_translation_domain' => false,
        ]);
    }
}
