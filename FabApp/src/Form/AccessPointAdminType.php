<?php

namespace App\Form;

use App\Entity\AccessPoint;
use App\Entity\Place;
use Symfony\Bridge\Doctrine\Form\Type\EntityType;
use Symfony\Component\Form\AbstractType;
use Symfony\Component\Form\Extension\Core\Type\ChoiceType;
use Symfony\Component\Form\Extension\Core\Type\SubmitType;
use Symfony\Component\Form\Extension\Core\Type\TextareaType;
use Symfony\Component\Form\Extension\Core\Type\TextType;
use Symfony\Component\Form\FormBuilderInterface;
use Symfony\Component\OptionsResolver\OptionsResolver;
use Symfony\Component\Validator\Constraints as Assert;

/**
 * Le formulaire d'un point d'accès — porte, portail, casier, zone.
 */
final class AccessPointAdminType extends AbstractType
{
    /**
     * ⚠️ Un champ absent d'ici retombe dans `form_rest()`, après le bouton
     * « Enregistrer » et sans thème : ajouté à l'écran sans que rien ne le
     * signale. Même contrat que `LoanableItemAdminType::SECTIONS`.
     */
    public const SECTIONS = [
        [
            'title' => 'access_points.section_identity',
            'fields' => ['nom', 'kind', 'description'],
        ],
        [
            /*
             * ⚠️ **`venue` et `place` sont deux questions différentes, et
             * voisines exprès.** Le lieu est LE SITE auquel la porte appartient —
             * c'est sur lui que les listes filtrent, et il est obligatoire.
             * `place` répond « qu'est-ce que ça ouvre », et vaut vide quand la
             * réponse n'existe pas : un portail d'entrée n'ouvre aucune salle en
             * particulier. Vide ne veut pas dire « pas encore saisi ».
             */
            'title' => 'access_points.section_location',
            'fields' => ['venue', 'place', 'localisation'],
        ],
    ];

    public function buildForm(FormBuilderInterface $builder, array $options): void
    {
        $builder
            ->add('nom', TextType::class, [
                'label' => 'access_points.field_name',
                'empty_data' => '',
                'help' => 'access_points.help_name',
                'constraints' => [
                    new Assert\NotBlank(message: 'Le nom est obligatoire.'),
                    new Assert\Length(max: 150, maxMessage: 'Le nom ne doit pas dépasser {{ limit }} caractères.'),
                ],
            ])
            /*
             * ⚠️ **Une liste FERMÉE, puisée dans la constante de l'entité.**
             * Recopier les quatre natures ici ferait deux listes à tenir
             * d'accord ; en texte libre, la colonne contiendrait « porte »,
             * « Porte », « door » et « entrée » au bout d'un mois, et plus aucun
             * regroupement ne serait possible.
             */
            ->add('kind', ChoiceType::class, [
                'label' => 'access_points.field_kind',
                'choices' => array_combine(
                    array_map(static fn (string $kind): string => 'access_points.kind_' . $kind, AccessPoint::KINDS),
                    AccessPoint::KINDS,
                ),
                'help' => 'access_points.help_kind',
            ])
            ->add('description', TextareaType::class, [
                'label' => 'access_points.field_description',
                'required' => false,
                'row_attr' => ['class' => 'full'],
                'help' => 'access_points.help_description',
            ])
            /*
             * 🔴 **`VenueChoiceType`, pas un `EntityType` de plus.** J'avais écrit
             * ici un `EntityType` avec `'choice_label' => 'nom'` — et `Venue`
             * expose `getName()`, pas `getNom()`, donc la page de création
             * jetait `NoSuchPropertyException`. Le lint était vert ; c'est
             * `app:render` avant redémarrage qui l'a dit.
             *
             * ⚠️ Et le type dédié apporte ce qu'un `EntityType` nu n'aurait
             * jamais eu : les lieux ARCHIVÉS restent listés mais désactivés,
             * parce qu'un enregistrement peut légitimement pointer sur un lieu
             * archivé et qu'un choix qui l'exclut rend cet enregistrement
             * INMODIFIABLE — sa valeur courante n'étant plus dans la liste.
             * Réécrire ça ici serait la deuxième vérité de la phase.
             */
            ->add('venue', VenueChoiceType::class, [
                'label' => 'access_points.field_venue',
                'help' => 'access_points.help_venue',
                'constraints' => [new Assert\NotNull(message: 'Le lieu est obligatoire.')],
            ])
            ->add('place', EntityType::class, [
                'label' => 'access_points.field_place',
                'class' => Place::class,
                'required' => false,
                // Un espace archivé ne se propose pas — même partage que partout.
                'query_builder' => static fn ($repository) => $repository->createQueryBuilder('p')
                    ->andWhere('p.archivedAt IS NULL')
                    ->orderBy('p.nom', 'ASC'),
                'choice_label' => 'nom',
                'placeholder' => 'access_points.ph_place',
                'help' => 'access_points.help_place',
            ])
            ->add('localisation', TextType::class, [
                'label' => 'access_points.field_localisation',
                'required' => false,
                'help' => 'access_points.help_localisation',
                'constraints' => [new Assert\Length(max: 150, maxMessage: 'Ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('save', SubmitType::class, ['label' => 'common.save']);
    }

    public function configureOptions(OptionsResolver $resolver): void
    {
        $resolver->setDefaults(['data_class' => AccessPoint::class]);
    }
}
