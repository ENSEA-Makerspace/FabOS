<?php

namespace App\Form;

use App\Entity\Machine;
use App\Entity\Material;
use Symfony\Bridge\Doctrine\Form\Type\EntityType;
use Symfony\Component\Form\AbstractType;
use Symfony\Component\Form\Extension\Core\Type\SubmitType;
use Symfony\Component\Form\Extension\Core\Type\TextareaType;
use Symfony\Component\Form\Extension\Core\Type\TextType;
use Symfony\Component\Form\Extension\Core\Type\UrlType;
use Symfony\Component\Form\FormBuilderInterface;
use Symfony\Component\OptionsResolver\OptionsResolver;
use Symfony\Component\Validator\Constraints as Assert;

final class MaterialAdminType extends AbstractType
{
    /**
     * **Le découpage de l'écran (S150)** — même contrat que
     * `MachineAdminType::SECTIONS`, lu par `_material_form.html.twig`.
     *
     * Un titre, ses champs dans l'ordre, `fold: true` pour ce qui passe
     * derrière un `<details>`. Onze champs à plat ne disaient pas lesquels
     * décrivent le matériau, lesquels disent où le trouver, et lesquels ne
     * servent qu'à l'illustrer.
     *
     * ⚠️ Un champ absent d'ici retombe dans `form_rest()`, après le bouton
     * « Enregistrer » et sans thème. Ajouter un `->add()` sans l'ajouter ici,
     * c'est le perdre à l'écran sans qu'aucun outil ne le signale.
     */
    public const SECTIONS = [
        [
            'title' => 'materials_form.section_identity',
            'fields' => ['name', 'category', 'color', 'description'],
        ],
        [
            'title' => 'materials_form.section_use',
            'fields' => ['specs', 'machines'],
        ],
        [
            // Deux questions qui vont ensemble : où il est ici, où on le
            // rachète. Côte à côte pour cette raison (règle 6).
            'title' => 'materials_form.section_supply',
            'fields' => ['storageLocation', 'purchaseUrl'],
        ],
        [
            // Repli : le catalogue se lit très bien sans image, et l'emoji est
            // le repli de l'image. Aucun des deux ne décide de rien.
            'title' => 'materials_form.section_media',
            'fold' => true,
            'fields' => ['imageUrl', 'icon'],
        ],
    ];

    public function buildForm(FormBuilderInterface $builder, array $options): void
    {
        // ⚠️ L'ordre des `->add()` suit SECTIONS, et `row_attr: {class: 'full'}`
        // est la seule façon de décider de la largeur d'un champ — le thème le
        // recopie sur l'enveloppe. Les seules paires côte à côte voulues ici
        // sont catégorie/couleur, rangement/rachat et image/emoji.
        $builder
            ->add('name', TextType::class, [
                'label' => 'Nom',
                'empty_data' => '',
                'row_attr' => ['class' => 'full'],
                'constraints' => [
                    new Assert\NotBlank(message: 'Le nom est obligatoire.'),
                    new Assert\Length(max: 150, maxMessage: 'Le nom ne doit pas dépasser {{ limit }} caractères.'),
                ],
            ])
            // ⚠️ L'exemple quitte le libellé pour l'aide : un libellé est le nom
            // du champ, pas sa notice. « Catégorie (ex : filament, plaque,
            // résine) » sur une étiquette en gras, c'est la notice qui crie.
            ->add('category', TextType::class, [
                'label' => 'Catégorie',
                'required' => false,
                'help' => 'materials_form.help_category',
                'constraints' => [new Assert\Length(max: 80)],
            ])
            ->add('color', TextType::class, [
                'label' => 'Couleur',
                'required' => false,
                'constraints' => [new Assert\Length(max: 60)],
            ])
            ->add('description', TextareaType::class, [
                'label' => 'Description',
                'required' => false,
                'row_attr' => ['class' => 'full'],
                'help' => 'materials_form.help_description',
                'constraints' => [new Assert\Length(max: 2000, maxMessage: 'La description ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('specs', TextareaType::class, [
                'label' => 'Spécifications',
                'required' => false,
                'row_attr' => ['class' => 'full'],
                'help' => 'materials_form.help_specs',
                'constraints' => [new Assert\Length(max: 2000)],
            ])
            // 🔴 **S151 — c'est `attr`, pas le gabarit, qui pose `.choice-grid`.**
            // Ce champ était « rendu à la main par le gabarit parce que les cases
            // cochables veulent leur `.choice-grid` » — et le rendre à la main
            // sautait `form_help()`, donc `materials_form.help_machines` n'était
            // affichée NULLE PART : la seule phrase qui dit que le matériau
            // apparaîtra sur la fiche des machines cochées.
            // Un `expanded` + `multiple` rend un conteneur `<div>` qui reçoit
            // `attr` (`widget_container_attributes`) : la classe atterrit au bon
            // endroit et `form_row()` redevient utilisable, avec son aide, son
            // `aria-describedby` et son bloc d'erreurs.
            ->add('machines', EntityType::class, [
                'label' => 'Machines qui acceptent ce matériau',
                'class' => Machine::class,
                'choice_label' => 'nom',
                'multiple' => true,
                'expanded' => true,
                'required' => false,
                'by_reference' => false,
                'row_attr' => ['class' => 'full'],
                'attr' => ['class' => 'choice-grid'],
                'help' => 'materials_form.help_machines',
                'query_builder' => static fn ($repo) => $repo->createQueryBuilder('machine')->orderBy('machine.nom', 'ASC'),
            ])
            ->add('storageLocation', TextType::class, [
                'label' => 'Emplacement de stockage',
                'required' => false,
                'help' => 'materials_form.help_storage',
                'constraints' => [new Assert\Length(max: 180)],
            ])
            ->add('purchaseUrl', UrlType::class, [
                'label' => 'Lien d’achat',
                'required' => false,
                'default_protocol' => 'https',
                'help' => 'materials_form.help_purchase',
                'constraints' => [new Assert\Length(max: 500), new Assert\Url(message: 'URL invalide.')],
            ])
            // ⚠️ « (optionnel) » disparaît des libellés : règle 5 de
            // `docs/FORM-DESIGN.md`, l'absence de mention EST la mention. Le
            // mot ne se met que sur les champs obligatoires.
            ->add('imageUrl', UrlType::class, [
                'label' => 'URL de l’image',
                'required' => false,
                'default_protocol' => 'https',
                'help' => 'materials_form.help_image',
                'constraints' => [new Assert\Length(max: 500), new Assert\Url(message: 'URL invalide.')],
            ])
            ->add('icon', TextType::class, [
                'label' => 'Icône emoji',
                'required' => false,
                'help' => 'materials_form.help_icon',
                'constraints' => [new Assert\Length(max: 16)],
            ])
            // ⚠️ S151 — `common.save` et non « Enregistrer » : ce bouton est le seul
            // de l'écran, et il était le dernier libellé français en dur d'un
            // formulaire qui déroule maintenant ses sections traduites.
            ->add('save', SubmitType::class, ['label' => 'common.save']);
    }

    public function configureOptions(OptionsResolver $resolver): void
    {
        $resolver->setDefaults([
            'data_class' => Material::class,
        ]);
    }
}
