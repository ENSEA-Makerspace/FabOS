<?php

namespace App\Form\Admin;

use App\Media\SiteMediaLibrary;
use Symfony\Component\Form\AbstractType;
use Symfony\Component\Form\Extension\Core\Type\ChoiceType;
use Symfony\Component\Form\Extension\Core\Type\TextType;
use Symfony\Component\Form\FormBuilderInterface;
use Symfony\Component\OptionsResolver\OptionsResolver;
use Symfony\Component\Validator\Constraints as Assert;

/**
 * `/admin/themes`, le brouillon de thème (S148, J-22).
 *
 * 🔴 **Les trois refus de `ThemeManager::saveDraft()` étaient des exceptions dont
 * le message partait en flash, suivies d'une redirection : les quatre champs
 * étaient perdus.** Une couleur tapée « 9E1B56 » sans le dièse effaçait donc
 * aussi le nom de l'organisation, le nom du lieu et le fichier de logo. Les mêmes
 * règles sont ici, en contraintes, sur leur champ.
 *
 * ⚠️ **`ThemeManager` garde ses `throw`, délibérément.** C'est lui le point de
 * passage : un import, une commande ou un autre écran doivent buter sur la même
 * règle. Ce que le type ajoute, c'est de la dire AVANT, là où l'opérateur tape.
 *
 * ⚠️ `orgName` et `venueLabel` étaient TRONQUÉS à 80 par `mb_substr()` ; la
 * contrainte les refuse. La borne reste 80 des deux côtés.
 */
final class ThemeDraftType extends AbstractType
{
    public function __construct(private readonly SiteMediaLibrary $media)
    {
    }

    public function buildForm(FormBuilderInterface $builder, array $options): void
    {
        $builder
            ->add('orgName', TextType::class, [
                'label' => 'admin_themes.org_name',
                'empty_data' => '',
                'attr' => ['maxlength' => 80],
                'constraints' => [
                    new Assert\NotBlank(message: 'Les deux noms publics sont obligatoires.'),
                    new Assert\Length(max: 80, maxMessage: 'Ce champ ne doit pas dépasser {{ limit }} caractères.'),
                ],
            ])
            ->add('venueLabel', TextType::class, [
                'label' => 'admin_themes.venue_label',
                'empty_data' => '',
                'attr' => ['maxlength' => 80],
                'constraints' => [
                    new Assert\NotBlank(message: 'Les deux noms publics sont obligatoires.'),
                    new Assert\Length(max: 80, maxMessage: 'Ce champ ne doit pas dépasser {{ limit }} caractères.'),
                ],
            ])
            ->add('primaryColor', TextType::class, [
                'label' => 'admin_themes.primary_color',
                'required' => false,
                'attr' => ['maxlength' => 7, 'placeholder' => '#9E1B56'],
                'constraints' => [new Assert\Regex(
                    pattern: '/^#(?:[0-9a-f]{3}|[0-9a-f]{6})$/i',
                    message: 'La couleur doit être un code hexadécimal, par exemple #9E1B56.',
                )],
            ])
            /*
             * 🔴 **Une LISTE, plus un champ de texte (S165).** Le champ demandait
             * le nom d'un fichier qui devait déjà se trouver sur le serveur : on
             * pouvait donc y taper n'importe quoi, et la seule façon d'y mettre
             * une valeur juste était d'avoir un accès SSH. Une liste ne peut
             * proposer que ce qui existe.
             * ⚠️ **Vide reste une réponse** — c'est « le logo livré », qui est le
             * cas normal, pas un champ qu'on aurait oublié de remplir.
             */
            ->add('logoPath', ChoiceType::class, [
                'label' => 'admin_themes.logo_path',
                'required' => false,
                'placeholder' => 'admin_themes.logo_none',
                'choices' => $this->logoChoices(),
            ]);
    }

    /**
     * ⚠️ **L'étiquette est le nom d'ORIGINE du fichier**, pas son nom sur disque.
     * Le second est un identifiant tiré au sort : lisible par la machine,
     * illisible pour la personne qui a envoyé `logo-2026-final.png`.
     *
     * @return array<string, string>
     */
    private function logoChoices(): array
    {
        $choices = [];
        foreach ($this->media->all() as $row) {
            $label = trim((string) ($row['originalName'] ?? '')) ?: (string) $row['filename'];
            // ⚠️ Deux fichiers peuvent porter le même nom d'origine : on suffixe
            // avec les dimensions plutôt que d'en perdre un silencieusement — un
            // tableau PHP écraserait la clé en double sans rien dire.
            if (isset($choices[$label])) {
                $label .= ' (' . ($row['width'] ?? '?') . '×' . ($row['height'] ?? '?') . ')';
            }
            $choices[$label] = (string) $row['mediaId'];
        }

        return $choices;
    }

    public function configureOptions(OptionsResolver $resolver): void
    {
        $resolver->setDefaults(['data_class' => null, 'csrf_token_id' => 'admin_themes']);
    }
}
