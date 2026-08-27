<?php

namespace App\Form\UsageRights;

use Symfony\Component\Form\AbstractType;
use Symfony\Component\Form\Extension\Core\Type\ChoiceType;
use Symfony\Component\Form\Extension\Core\Type\TextType;
use Symfony\Component\Form\Extension\Core\Type\TimeType;
use Symfony\Component\Form\FormBuilderInterface;
use Symfony\Component\OptionsResolver\Options;
use Symfony\Component\OptionsResolver\OptionsResolver;
use Symfony\Component\Validator\Constraints as Assert;

/**
 * Un grant v2, avec sa première plage horaire (S147, J-22).
 *
 * ⚠️ **Dix champs qui posent UNE phrase** : « ce package laisse *utiliser* les
 * *imprimantes 3D* du *lieu X*, le *jeudi* de *14 h* à *18 h* ». Le formulaire
 * les garde ensemble parce que c'est ainsi qu'un opérateur y pense — la carte
 * dit la phrase, pas les colonnes d'une table.
 *
 * 🔴 **S149 — neuf champs posés en égaux, et sept avaient un défaut sensé.**
 * C'était le vrai coût de cet éditeur : pas les clics, mais neuf contrôles
 * alignés dont deux seulement doivent être remplis. Deux réponses, et aucune ne
 * touche au modèle stocké :
 *   · `window_preset` remplace les TROIS champs de plage dans le cas courant —
 *     « à toute heure » (défaut) · « personnalisé… ». ⚠️ **Deux choix, pas trois :**
 *     « horaires d'ouverture » a été proposé puis RETIRÉ (S149), parce qu'il ne
 *     pouvait que COPIER la grille du lieu au moment de la soumission et que la
 *     copie ne suit pas un changement d'horaires. Le pourquoi complet est dans
 *     `UsageRightsAdminController` au-dessus de `$presetChoices` ; ne pas le
 *     redécrire ici, et ne pas le remettre sans lire ce paragraphe.
 *     C'est une couche d'INTERFACE : `day_of_week` / `start_time` / `end_time`
 *     restent les champs qui écrivent, et le préréglage dit seulement lesquels
 *     comptent. Voir `UsageRightsAdminController::edit()`.
 *   · Le gabarit replie `venue_id` / `section` / `reservable` / `category_label`
 *     derrière une ligne qui dit la portée en toutes lettres (règle 7).
 *
 * 🔴 **Un champ replié POSTE quand même.** Le repli du gabarit et le contrôleur
 * Stimulus `conditional-field` CACHENT, ils ne désactivent rien : les trois
 * champs de plage arrivent toujours au contrôleur, remplis de leurs défauts.
 * C'est donc `window_preset`, et lui seul, qui décide si on écrit une plage —
 * lire la valeur de `day_of_week` pour en déduire l'intention réinventerait le
 * bug que le préréglage vient supprimer.
 *
 * 🔴 **La plage est FACULTATIVE et son absence n'est pas neutre** : un grant sans
 * plage ouvre toute la semaine. C'est pour ça que les trois champs de temps sont
 * `required => false` et que rien ici ne les impose — mais aussi pourquoi le
 * contrôleur, s'il en reçoit une incomplète, garde le grant et signale la plage
 * plutôt que de tout refuser : le grant est la partie qui compte.
 *
 * ⚠️ Les heures rendent une CHAÎNE `H:i`, comme aujourd'hui : `GrantWindow::
 * fromClock()` les lit ainsi, et c'est lui qui sait que minuit en fin de plage
 * vaut 1440 et non 0.
 */
final class PackageGrantType extends AbstractType
{
    public function buildForm(FormBuilderInterface $builder, array $options): void
    {
        $builder
            ->add('feature', ChoiceType::class, [
                'label' => 'usage_rights.grant_feature',
                'row_attr' => ['class' => 'afp-select'],
                'label_attr' => ['class' => 'afp-k'],
                'choice_translation_domain' => false,
                'choices' => $options['feature_choices'],
                'constraints' => [new Assert\NotBlank(message: 'Choisissez une fonctionnalité.')],
            ])
            ->add('grant_action', ChoiceType::class, [
                'label' => 'usage_rights.grant_action',
                'row_attr' => ['class' => 'afp-select'],
                'label_attr' => ['class' => 'afp-k'],
                'choice_translation_domain' => false,
                'choices' => $options['action_choices'],
            ])
            // ⚠️ **Le préréglage de plage (S149).** DEUX choix là où l'écran
            // posait trois contrôles : le cas courant — un grant éveillé tout le
            // temps — devient UNE liste laissée sur son défaut, et les trois
            // champs de plage ne se montrent que sur « personnalisé… ».
            // ⚠️ Il n'a PAS de colonne : rien n'est stocké de ce champ. Le
            // contrôleur le lit pour savoir quelles plages écrire, puis l'oublie.
            // Un grant relu depuis la base n'a donc pas de préréglage à
            // retrouver — c'est un champ de saisie, pas un champ de modèle.
            ->add('window_preset', ChoiceType::class, [
                'label' => 'usage_rights.grant_window',
                'row_attr' => ['class' => 'afp-select'],
                'label_attr' => ['class' => 'afp-k'],
                'choice_translation_domain' => false,
                'choices' => $options['window_preset_choices'],
                // Même raison qu'en dessous : sans ça un `ChoiceType` non requis
                // s'ajoute une option VIDE en tête, et c'est elle qui serait
                // présélectionnée à la place de « à toute heure ».
                'placeholder' => false,
                'required' => false,
            ])
            ->add('venue_id', ChoiceType::class, [
                'label' => 'venue_context.menu',
                'row_attr' => ['class' => 'afp-select'],
                'label_attr' => ['class' => 'afp-k'],
                'choice_translation_domain' => false,
                'choices' => $options['venue_choices'],
                'required' => false,
                'placeholder' => 'usage_rights.grant_any_venue',
            ])
            ->add('section', TextType::class, [
                'label' => 'usage_rights.grant_section',
                'row_attr' => ['class' => 'afp-select'],
                'label_attr' => ['class' => 'afp-k'],
                'required' => false,
                'empty_data' => '',
                'constraints' => [new Assert\Length(max: 80, maxMessage: 'Ce champ ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('reservable', ChoiceType::class, [
                'label' => 'usage_rights.grant_resource',
                'row_attr' => ['class' => 'afp-select'],
                'label_attr' => ['class' => 'afp-k'],
                'choice_translation_domain' => false,
                'choices' => $options['resource_choices'],
                'required' => false,
                'placeholder' => 'usage_rights.grant_any_resource',
            ])
            ->add('category_label', ChoiceType::class, [
                'label' => 'usage_rights.grant_category',
                'row_attr' => ['class' => 'afp-select'],
                'label_attr' => ['class' => 'afp-k'],
                'choice_translation_domain' => false,
                'choices' => $options['category_choices'],
                'required' => false,
                'placeholder' => 'usage_rights.grant_any_category',
            ])
            // 🔴 **S151 — ce champ portait le libellé du PRÉRÉGLAGE**
            // (`usage_rights.grant_window`, « Créneau »), c'est-à-dire le même mot
            // que la liste juste au-dessus. Deux contrôles côte à côte, un seul
            // nom : la question « quel créneau ? » était posée deux fois et le
            // jour ne se nommait nulle part.
            ->add('day_of_week', ChoiceType::class, [
                'label' => 'usage_rights.grant_day',
                'row_attr' => ['class' => 'afp-select'],
                'label_attr' => ['class' => 'afp-k'],
                'choice_translation_domain' => false,
                'choices' => $options['day_choices'],
                // ⚠️ `placeholder => false` et non l'oubli du défaut : sans lui,
                // un `ChoiceType` non requis s'ajoute une option VIDE en tête, et
                // c'est elle qui serait présélectionnée à la place du premier
                // jour.
                // ⚠️ **S149 — la liste ne porte plus « À toute heure » (le `0`).**
                // Ce choix disait la même chose que le préréglage « à toute
                // heure », à un autre endroit : deux contrôles pour une décision,
                // et une combinaison — « personnalisé » + « à toute heure » — qui
                // ne voulait rien dire. Le `0` reste accepté côté contrôleur pour
                // qu'un POST ancien ne casse pas, il n'est simplement plus offert.
                'placeholder' => false,
                'required' => false,
            ])
            ->add('start_time', TimeType::class, $this->clock() + ['label' => 'usage_rights.window_from'])
            ->add('end_time', TimeType::class, $this->clock() + ['label' => 'usage_rights.window_to']);
    }

    /** @return array<string, mixed> */
    private function clock(): array
    {
        return [
            'widget' => 'single_text', 'html5' => true, 'input' => 'string', 'input_format' => 'H:i', 'required' => false,
            'row_attr' => ['class' => 'afp-select'], 'label_attr' => ['class' => 'afp-k'],
        ];
    }

    public function configureOptions(OptionsResolver $resolver): void
    {
        $resolver
            ->setDefaults(['data_class' => null])
            ->setRequired(['feature_choices', 'action_choices', 'venue_choices', 'resource_choices', 'category_choices', 'day_choices', 'window_preset_choices', 'package_key'])
            ->setDefault('csrf_token_id', static fn (Options $o): string => 'usage_package_grants_' . $o['package_key']);
    }
}
