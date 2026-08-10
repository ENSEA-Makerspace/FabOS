<?php

namespace App\Form;

use App\Entity\Venue;
use Symfony\Component\Form\AbstractType;
use Symfony\Component\Form\Extension\Core\Type\SubmitType;
use Symfony\Component\Form\Extension\Core\Type\TextType;
use Symfony\Component\Form\Extension\Core\Type\TimezoneType;
use Symfony\Component\Form\FormBuilderInterface;
use Symfony\Component\OptionsResolver\OptionsResolver;

/**
 * Create/edit form for a sub-venue (S129).
 *
 * ⚠️ **Labels are translation keys, unlike the older admin form types.** Those
 * predate the five-catalogue rule and carry French literals; every string added
 * here is new, so it starts compliant rather than adding to the backlog.
 */
final class VenueType extends AbstractType
{
    public function buildForm(FormBuilderInterface $builder, array $options): void
    {
        $builder
            ->add('name', TextType::class, [
                'label' => 'venues.field.name',
                'empty_data' => '',
                'attr' => ['autofocus' => true],
            ])
            ->add('address', TextType::class, [
                'label' => 'venues.field.address',
                'required' => false,
                'help' => 'venues.help.address',
            ])
            ->add('timezone', TimezoneType::class, [
                'label' => 'venues.field.timezone',
                'help' => 'venues.help.timezone',
            ]);

        // ⚠️ Write-once. The slug is what `?location=` carries, so it lives in
        // shared admin URLs and saved member links; renaming it 404s them. It is
        // offered at creation and is absent — not disabled — afterwards, because a
        // disabled field still round-trips through the request on some browsers.
        if ($options['with_slug']) {
            $builder->add('slug', TextType::class, [
                'label' => 'venues.field.slug',
                'empty_data' => '',
                'help' => 'venues.help.slug',
            ]);
        }

        $builder->add('save', SubmitType::class, ['label' => 'common.save']);
    }

    public function configureOptions(OptionsResolver $resolver): void
    {
        $resolver->setDefaults([
            'data_class' => Venue::class,
            'with_slug' => false,
        ]);
        $resolver->setAllowedTypes('with_slug', 'bool');
    }
}
