<?php

namespace App\Form;

use App\Entity\Badge;
use App\Entity\Formation;
use Symfony\Bridge\Doctrine\Form\Type\EntityType;
use Symfony\Component\Form\AbstractType;
use Symfony\Component\Form\Extension\Core\Type\CheckboxType;
use Symfony\Component\Form\Extension\Core\Type\IntegerType;
use Symfony\Component\Form\Extension\Core\Type\SubmitType;
use Symfony\Component\Form\Extension\Core\Type\TextareaType;
use Symfony\Component\Form\Extension\Core\Type\TextType;
use Symfony\Component\Form\FormBuilderInterface;
use Symfony\Component\OptionsResolver\OptionsResolver;
use Symfony\Component\Validator\Constraints as Assert;

final class FormationAdminType extends AbstractType
{
    /**
     * L'ordre et le découpage de l'écran — motif `SECTIONS` (S151, R3), déroulé
     * par `site/_form_sections.html.twig`. Une seule liste, dans le formulaire,
     * au lieu d'une par gabarit qui rendait le même type.
     *
     * 🔴 **Ce gabarit dessinait ses rangées à la main** (`form_label` +
     * `form_widget`), donc il sautait le thème : le champ obligatoire n'était
     * marqué nulle part — 1 champ requis, **0 mention « requis »** à l'écran,
     * mesuré avant conversion. La règle 5 de S149 existe depuis, dans le thème.
     */
    public const SECTIONS = [
        [
            'title' => 'admin_formation_form.section_identity',
            'fields' => ['titre', 'description', 'categorie', 'niveau'],
        ],
        [
            'title' => 'admin_formation_form.section_session',
            'fields' => ['duree', 'formateur', 'placesTotales'],
        ],
        [
            'title' => 'admin_formation_form.section_content',
            'fold' => true,
            'fields' => ['objectifs', 'prerequis', 'materielFourni'],
        ],
        [
            /*
             * ⚠️ **`requiresPractical` est dans la section du BADGE, pas dans
             * une section « sécurité » à elle.** Les deux répondent à la même
             * question — qu'est-ce qu'on obtient, et à quelle condition — et les
             * séparer ferait cocher l'un sans voir l'autre. C'est la règle 6 :
             * ce qui se décide ensemble se montre ensemble.
             * 🔴 Et il est ICI parce que `tools/form_fields.py` refuse un champ
             * déclaré et absent de `SECTIONS` : il sortirait par `form_rest()`,
             * après « Enregistrer » et sans thème.
             */
            'title' => 'admin_formation_form.section_media',
            'fold' => true,
            'fields' => ['image', 'badge', 'requiresPractical'],
        ],
    ];

    public function buildForm(FormBuilderInterface $builder, array $options): void
    {
        $builder
            ->add('titre', TextType::class, [
                'label' => 'form.title',
                'empty_data' => '',
                'constraints' => [
                    new Assert\NotBlank(message: 'Le titre est obligatoire.'),
                    new Assert\Length(max: 255, maxMessage: 'Le titre ne doit pas dépasser {{ limit }} caractères.'),
                ],
            ])
            ->add('description', TextareaType::class, [
                'row_attr' => ['class' => 'full'],
                'label' => 'form.description',
                'required' => false,
                'constraints' => [new Assert\Length(max: 3000, maxMessage: 'La description ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('image', TextType::class, [
                'label' => 'admin_formation_form.image',
                'required' => false,
                'constraints' => [new Assert\Length(max: 255, maxMessage: "L'image ne doit pas dépasser {{ limit }} caractères.")],
            ])
            ->add('categorie', TextType::class, [
                'label' => 'form.category',
                'required' => false,
                // ⚠️ La liste est proposée, pas imposée — `list` et non un
                // `ChoiceType` : une catégorie neuve doit rester possible.
                'attr' => ['list' => 'formation-category-options'],
                'help' => 'admin_formation_form.help_categorie',
                'constraints' => [new Assert\Length(max: 100, maxMessage: 'La catégorie ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('niveau', IntegerType::class, [
                'label' => 'form.level',
                'required' => false,
                'constraints' => [new Assert\Range(notInRangeMessage: 'Le niveau doit être compris entre {{ min }} et {{ max }}.', min: 1, max: 3)],
            ])
            ->add('duree', TextType::class, [
                'label' => 'admin_formation_form.duree',
                'required' => false,
                'constraints' => [new Assert\Length(max: 100, maxMessage: 'La durée ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('formateur', TextType::class, [
                'label' => 'admin_formation_form.formateur',
                'required' => false,
                'constraints' => [new Assert\Length(max: 150, maxMessage: 'Le formateur ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('placesTotales', IntegerType::class, [
                'label' => 'admin_formation_form.places_totales',
                'required' => false,
                'constraints' => [
                    new Assert\PositiveOrZero(message: 'Le nombre de places doit être un entier positif.'),
                    new Assert\LessThanOrEqual(value: 1000, message: 'Le nombre de places est trop élevé.'),
                ],
            ])
            ->add('objectifs', TextareaType::class, [
                'row_attr' => ['class' => 'full'],
                'label' => 'admin_formation_form.objectifs',
                'required' => false,
                'constraints' => [new Assert\Length(max: 3000, maxMessage: 'Les objectifs ne doivent pas dépasser {{ limit }} caractères.')],
            ])
            ->add('prerequis', TextareaType::class, [
                'row_attr' => ['class' => 'full'],
                'label' => 'form.prerequisites',
                'required' => false,
                'constraints' => [new Assert\Length(max: 3000, maxMessage: 'Les prérequis ne doivent pas dépasser {{ limit }} caractères.')],
            ])
            ->add('materielFourni', TextareaType::class, [
                'row_attr' => ['class' => 'full'],
                'label' => 'admin_formation_form.materiel_fourni',
                'required' => false,
                'constraints' => [new Assert\Length(max: 3000, maxMessage: 'Le matériel fourni ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('badge', EntityType::class, [
                'label' => 'admin_formation_form.badge',
                'class' => Badge::class,
                'choice_label' => static fn (Badge $badge): string => sprintf('%s (#%d)', $badge->getNom(), $badge->getId()),
                'placeholder' => 'admin_formation_form.ph_badge',
                'required' => false,
            ])
            /*
             * 🔴 **S180b — l'exigence de validation pratique se DÉCLARE.** Elle se
             * devinait par mots-clés français (`laser`, `soudure`, `fraiseuse`,
             * `cnc`, `brodeuse`) cherchés dans le titre : une garde de SÉCURITÉ
             * décidée par une correspondance de chaîne. « Découpe au CO2 »,
             * « Plasma », « Tour à métaux » ou tout intitulé anglais n'exigeait
             * rien, silencieusement.
             *
             * ⚠️ `false` est un AVIS et il gagne sur les mots-clés — c'est ce qui
             * permet de décocher une formation qu'ils attrapent à tort. La case
             * n'est donc jamais « vide » une fois enregistrée.
             */
            ->add('requiresPractical', CheckboxType::class, [
                'label' => 'admin_formation_form.requires_practical',
                'help' => 'admin_formation_form.help_requires_practical',
                'required' => false,
                'row_attr' => ['class' => 'full'],
            ])
            ->add('save', SubmitType::class, ['label' => 'common.save']);
    }

    public function configureOptions(OptionsResolver $resolver): void
    {
        $resolver->setDefaults([
            'data_class' => Formation::class,
        ]);
    }
}
