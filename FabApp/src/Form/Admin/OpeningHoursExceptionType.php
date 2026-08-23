<?php

namespace App\Form\Admin;

use Symfony\Component\Form\AbstractType;
use Symfony\Component\Form\Extension\Core\Type\CheckboxType;
use Symfony\Component\Form\Extension\Core\Type\DateType;
use Symfony\Component\Form\Extension\Core\Type\TextType;
use Symfony\Component\Form\Extension\Core\Type\TimeType;
use Symfony\Component\Form\FormBuilderInterface;
use Symfony\Component\OptionsResolver\OptionsResolver;
use Symfony\Component\Validator\Constraints as Assert;
use Symfony\Component\Validator\Context\ExecutionContextInterface;

/**
 * `/admin/horaires`, « ajouter une exception » (S148, J-22).
 *
 * ⚠️ **Un écran, trois formulaires, et un seul se convertit.** Le sélecteur de
 * portée est un filtre GET — sa place est dans l'URL et elle y est déjà. La
 * semaine est une MATRICE (`open_2[]`, `close_2[]` : une ligne peut porter
 * plusieurs plages et la sauvegarde remplace le jour en bloc). Reste celui-ci,
 * qui est bien une liste de champs.
 *
 * 🔴 **Les quatre refus étaient des phrases françaises en dur affichées en haut
 * de page.** Ils sont maintenant des contraintes, donc traduits par le catalogue
 * `validators` et posés SUR le champ fautif — et surtout la page ne redirige
 * plus : elle re-rendait déjà en 422, mais avec un formulaire vide.
 *
 * ⚠️ Les dates et les heures restent des CHAÎNES (`input: 'string'`). Ce sont des
 * heures d'horloge murale que le contrôleur construit dans le fuseau du labo ;
 * laisser Symfony hydrater un objet le ferait dans le fuseau PHP (UTC ici) et
 * décalerait silencieusement une fermeture d'une heure.
 */
final class OpeningHoursExceptionType extends AbstractType
{
    public function buildForm(FormBuilderInterface $builder, array $options): void
    {
        $builder
            ->add('exception_date', DateType::class, [
                'label' => 'hours.exception_date',
                'widget' => 'single_text',
                'html5' => true,
                'input' => 'string',
                'input_format' => 'Y-m-d',
                'constraints' => [new Assert\NotBlank(message: 'Date invalide.')],
            ])
            ->add('exception_end', DateType::class, [
                'label' => 'hours.exception_end',
                'widget' => 'single_text',
                'html5' => true,
                'input' => 'string',
                'input_format' => 'Y-m-d',
                'required' => false,
            ])
            ->add('exception_reason', TextType::class, [
                'label' => 'hours.exception_reason',
                'required' => false,
                'attr' => ['maxlength' => 120, 'placeholder' => $options['reason_placeholder']],
                'constraints' => [new Assert\Length(max: 120, maxMessage: 'Ce champ ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('exception_closed', CheckboxType::class, [
                'label' => 'hours.exception_closed',
                'required' => false,
            ])
            ->add('exception_open', TimeType::class, $this->clock() + ['label' => 'hours.col_open'])
            ->add('exception_close', TimeType::class, $this->clock() + ['label' => 'hours.col_close']);
    }

    /** @return array<string, mixed> */
    private function clock(): array
    {
        return [
            'widget' => 'single_text',
            'html5' => true,
            'input' => 'string',
            'input_format' => 'H:i',
            'with_seconds' => false,
            'required' => false,
        ];
    }

    public function configureOptions(OptionsResolver $resolver): void
    {
        $resolver
            ->setDefaults([
                'data_class' => null,
                'csrf_token_id' => 'admin_opening_hours',
                'reason_placeholder' => '',
                // ⚠️ Les deux règles qui portent sur PLUSIEURS champs vivent au
                // niveau du formulaire : une contrainte de champ ne voit pas ses
                // voisins, et c'est exactement ce qu'il faut savoir ici.
                'constraints' => [new Assert\Callback([self::class, 'validateRanges'])],
            ])
            ->setAllowedTypes('reason_placeholder', 'string');
    }

    /**
     * ⚠️ `atPath('[champ]')` — avec des crochets, parce que la donnée du
     * formulaire est un TABLEAU. Sans eux la violation se pose sur le formulaire
     * entier et l'erreur remonte en haut de page, ce qu'on vient justement de
     * quitter.
     *
     * @param array<string, mixed>|null $data
     */
    public static function validateRanges(?array $data, ExecutionContextInterface $context): void
    {
        if ($data === null) {
            return;
        }

        $start = (string) ($data['exception_date'] ?? '');
        $end = (string) ($data['exception_end'] ?? '');
        // ⚠️ Une fin AVANT le début, pas une fin ÉGALE : vide comme égal veut dire
        // « un seul jour », et c'est le cas courant.
        if ($start !== '' && $end !== '' && $end < $start) {
            $context->buildViolation('La fin de la fermeture doit venir après son début.')
                ->atPath('[exception_end]')
                ->addViolation();
        }

        if ((bool) ($data['exception_closed'] ?? false)) {
            return;
        }

        $open = (string) ($data['exception_open'] ?? '');
        $close = (string) ($data['exception_close'] ?? '');
        if ($open === '' || $close === '' || $close <= $open) {
            $context->buildViolation('Une ouverture exceptionnelle demande une heure de début et une heure de fin valides.')
                ->atPath('[exception_open]')
                ->addViolation();
        }
    }
}
