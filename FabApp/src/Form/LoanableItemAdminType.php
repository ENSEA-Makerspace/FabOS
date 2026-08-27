<?php

namespace App\Form;

use App\Entity\LoanableItem;
use Symfony\Component\Form\AbstractType;
use Symfony\Component\Form\Extension\Core\Type\IntegerType;
use Symfony\Component\Form\Extension\Core\Type\SubmitType;
use Symfony\Component\Form\Extension\Core\Type\TextareaType;
use Symfony\Component\Form\Extension\Core\Type\TextType;
use Symfony\Component\Form\Extension\Core\Type\UrlType;
use Symfony\Component\Form\FormBuilderInterface;
use Symfony\Component\OptionsResolver\OptionsResolver;
use Symfony\Component\Validator\Constraints as Assert;

final class LoanableItemAdminType extends AbstractType
{
    /**
     * **Le découpage de l'écran (S150)** — même contrat que
     * `MachineAdminType::SECTIONS`, lu par `_loanable_item_form.html.twig`.
     *
     * ⚠️ Un champ absent d'ici retombe dans `form_rest()`, après le bouton
     * « Enregistrer » et sans thème. Ajouter un `->add()` sans l'ajouter ici,
     * c'est le perdre à l'écran sans qu'aucun outil ne le signale.
     */
    public const SECTIONS = [
        [
            'title' => 'loans.item_section_identity',
            'fields' => ['name', 'category', 'description'],
        ],
        [
            // ⚠️ `venue` et `storageLocation` sont deux questions différentes et
            // voisines exprès : le lieu est LE SITE auquel l'objet appartient
            // (c'est sur lui que les listes filtrent), `storageLocation` est
            // l'endroit DEDANS — le tiroir, l'étagère, l'armoire. Aucune ne
            // remplace l'autre, et c'est parce qu'elles vont ensemble qu'elles
            // sont côte à côte (règle 6).
            'title' => 'loans.item_section_stock',
            'fields' => ['quantity', 'venue', 'storageLocation'],
        ],
        [
            // Repli : le catalogue de prêt se lit sans image, et l'emoji est le
            // repli de l'image. Aucun des deux ne décide de rien.
            'title' => 'loans.item_section_media',
            'fold' => true,
            'fields' => ['imageUrl', 'icon'],
        ],
    ];

    public function buildForm(FormBuilderInterface $builder, array $options): void
    {
        // ⚠️ L'ordre des `->add()` suit SECTIONS, et `row_attr: {class: 'full'}`
        // est la seule façon de décider de la largeur d'un champ — le thème le
        // recopie sur l'enveloppe. Les seules paires côte à côte voulues ici
        // sont lieu/rangement et image/emoji.
        $builder
            ->add('name', TextType::class, [
                'label' => 'Nom',
                'empty_data' => '',
                'row_attr' => ['class' => 'full'],
                'constraints' => [
                    new Assert\NotBlank(message: 'Le nom est obligatoire.'),
                    new Assert\Length(max: 150),
                ],
            ])
            // ⚠️ L'exemple quitte le libellé pour l'aide : un libellé est le nom
            // du champ, pas sa notice.
            ->add('category', TextType::class, [
                'label' => 'Catégorie',
                'required' => false,
                'help' => 'loans.item_help_category',
                'constraints' => [new Assert\Length(max: 80)],
            ])
            ->add('description', TextareaType::class, [
                'label' => 'Description',
                'required' => false,
                'row_attr' => ['class' => 'full'],
                'help' => 'loans.item_help_description',
                'constraints' => [new Assert\Length(max: 2000)],
            ])
            ->add('quantity', IntegerType::class, [
                'label' => 'Quantité',
                'empty_data' => '1',
                'row_attr' => ['class' => 'full'],
                'help' => 'loans.item_help_quantity',
                'constraints' => [new Assert\PositiveOrZero(message: 'La quantité doit être positive.')],
            ])
            // `VenueChoiceType` porte déjà son libellé ET son aide (`venues.help.venue`).
            ->add('venue', VenueChoiceType::class)
            ->add('storageLocation', TextType::class, [
                'label' => 'Emplacement de stockage',
                'required' => false,
                'help' => 'loans.item_help_storage',
                'constraints' => [new Assert\Length(max: 180)],
            ])
            // ⚠️ « (optionnel) » disparaît des libellés : règle 5 de
            // `docs/FORM-DESIGN.md`, l'absence de mention EST la mention.
            ->add('imageUrl', UrlType::class, [
                'label' => 'URL de l’image',
                'required' => false,
                'default_protocol' => 'https',
                'help' => 'loans.item_help_image',
                'constraints' => [new Assert\Length(max: 500), new Assert\Url(message: 'URL invalide.')],
            ])
            ->add('icon', TextType::class, [
                'label' => 'Icône emoji',
                'required' => false,
                'help' => 'loans.item_help_icon',
                'constraints' => [new Assert\Length(max: 16)],
            ])
            // ⚠️ S151 — `common.save` et non « Enregistrer » : ce bouton est le seul
            // de l'écran, et il était le dernier libellé français en dur d'un
            // formulaire qui déroule maintenant ses sections traduites.
            ->add('save', SubmitType::class, ['label' => 'common.save']);
    }

    public function configureOptions(OptionsResolver $resolver): void
    {
        $resolver->setDefaults(['data_class' => LoanableItem::class]);
    }
}
