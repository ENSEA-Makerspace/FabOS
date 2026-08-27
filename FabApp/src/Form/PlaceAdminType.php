<?php

namespace App\Form;

use App\Entity\Place;
use Symfony\Component\Form\AbstractType;
use Symfony\Component\Form\Extension\Core\Type\IntegerType;
use Symfony\Component\Form\Extension\Core\Type\SubmitType;
use Symfony\Component\Form\Extension\Core\Type\TextareaType;
use Symfony\Component\Form\Extension\Core\Type\TextType;
use Symfony\Component\Form\FormBuilderInterface;
use Symfony\Component\OptionsResolver\OptionsResolver;
use Symfony\Component\Validator\Constraints as Assert;

final class PlaceAdminType extends AbstractType
{
    /**
     * L'ordre et le découpage de l'écran — motif `SECTIONS` (S151, R3), déroulé
     * par `site/_form_sections.html.twig`.
     *
     * 🔴 **Trois champs de ce formulaire tombaient SOUS le bouton Enregistrer.**
     * `category`, `manager` et `department` étaient déclarés ici et nommés dans
     * AUCUN des deux gabarits : ils sortaient par `form_rest()`, mesurés à
     * y=995/1078/1162 alors que la rangée d'actions est à y=892. Un opérateur
     * remplissait l'écran, cliquait « Créer l'espace », et ne les voyait jamais.
     * ⚠️ Le pire est qu'ils avaient l'air NORMAUX — `form_rest()` passe par le
     * thème, donc ils portaient bien leur `.form-field` : rien ne signalait qu'ils
     * étaient au mauvais endroit, sauf de regarder la page.
     */
    public const SECTIONS = [
        [
            'title' => 'admin_place_form.section_identity',
            'fields' => ['nom', 'description'],
        ],
        [
            'title' => 'admin_place_form.section_where',
            'fields' => ['venue', 'localisation', 'capacite'],
        ],
        [
            // Repli : trois champs de classement interne, tous facultatifs, et
            // aucun ne décide de quoi que ce soit. C'est une seconde passe.
            'title' => 'admin_place_form.section_admin',
            'fold' => true,
            'fields' => ['category', 'manager', 'department'],
        ],
    ];

    public function buildForm(FormBuilderInterface $builder, array $options): void
    {
        $builder
            ->add('nom', TextType::class, [
                'label' => 'form.name',
                'empty_data' => '',
                'constraints' => [
                    new Assert\NotBlank(message: 'Le nom est obligatoire.'),
                    new Assert\Length(max: 150, maxMessage: 'Le nom ne doit pas dépasser {{ limit }} caractères.'),
                ],
            ])
            ->add('venue', VenueChoiceType::class)
            ->add('localisation', TextType::class, [
                'label' => 'form.location',
                'required' => false,
                'constraints' => [new Assert\Length(max: 150, maxMessage: 'Ce champ ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('capacite', IntegerType::class, [
                'label' => 'admin_place_form.capacite',
                'required' => false,
                'constraints' => [new Assert\PositiveOrZero(message: 'La capacité doit être positive.')],
            ])
            ->add('category', TextType::class, ['label' => 'form.category', 'required' => false, 'constraints' => [new Assert\Length(max: 120)]])
            ->add('manager', TextType::class, ['label' => 'admin_place_form.manager', 'required' => false, 'constraints' => [new Assert\Length(max: 150)]])
            ->add('department', TextType::class, ['label' => 'admin_place_form.department', 'required' => false, 'constraints' => [new Assert\Length(max: 150)]])
            ->add('description', TextareaType::class, [
                'label' => 'form.description',
                // ⚠️ La pleine largeur vient d'ICI, pas du gabarit — les deux
                // gabarits la re-décidaient chacun de leur côté, alors qu'un
                // commentaire de l'un affirmait déjà que c'était un fait du
                // formulaire.
                'row_attr' => ['class' => 'full'],
                'required' => false,
                'constraints' => [new Assert\Length(max: 2000, maxMessage: 'La description ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('save', SubmitType::class, ['label' => 'common.save']);
    }

    public function configureOptions(OptionsResolver $resolver): void
    {
        $resolver->setDefaults([
            'data_class' => Place::class,
        ]);
    }
}
