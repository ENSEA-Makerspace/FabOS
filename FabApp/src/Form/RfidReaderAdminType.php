<?php

namespace App\Form;

use App\Entity\AccessPoint;
use App\Entity\Machine;
use App\Entity\RfidReader;
use Symfony\Bridge\Doctrine\Form\Type\EntityType;
use Symfony\Component\Form\AbstractType;
use Symfony\Component\Form\Extension\Core\Type\CheckboxType;
use Symfony\Component\Form\Extension\Core\Type\SubmitType;
use Symfony\Component\Form\Extension\Core\Type\TextType;
use Symfony\Component\Form\FormBuilderInterface;
use Symfony\Component\Form\FormError;
use Symfony\Component\Form\FormEvent;
use Symfony\Component\Form\FormEvents;
use Symfony\Component\OptionsResolver\OptionsResolver;
use Symfony\Component\Validator\Constraints as Assert;

final class RfidReaderAdminType extends AbstractType
{
    public function buildForm(FormBuilderInterface $builder, array $options): void
    {
        $builder
            ->add('name', TextType::class, [
                'label' => 'admin_rfid_form.name',
                'empty_data' => '',
                'constraints' => [
                    new Assert\NotBlank(message: 'Le nom est obligatoire.'),
                    new Assert\Length(max: 120, maxMessage: 'Le nom ne doit pas dépasser {{ limit }} caractères.'),
                ],
            ])
            ->add('readerToken', TextType::class, [
                'label' => 'admin_rfid_form.reader_token',
                'required' => false,
                'empty_data' => '',
                'help' => 'admin_rfid_form.help_reader_token',
                'constraints' => [
                    new Assert\Length(max: 120, maxMessage: 'Le readerToken ne doit pas dépasser {{ limit }} caractères.'),
                    new Assert\Regex(pattern: '/^[a-zA-Z0-9_-]*$/', message: 'Utilisez uniquement lettres, chiffres, tirets et underscores.'),
                ],
            ])
            /*
             * 🔴 **S175 — `NotNull` est parti d'ici, et il n'est pas parti tout
             * court.** La contrainte disait « la machine associée est
             * obligatoire », ce qui était vrai tant qu'un boîtier ne pouvait
             * commander qu'une machine. La règle réelle est plus forte et plus
             * simple : **exactement une** des deux cibles. La mettre sur un seul
             * champ aurait rendu la porte impossible ; la retirer sans la
             * remplacer aurait permis un boîtier qui n'ouvre rien.
             */
            ->add('machine', EntityType::class, [
                'label' => 'admin_rfid_form.machine',
                'class' => Machine::class,
                'required' => false,
                'choice_label' => static fn (Machine $machine): string => sprintf('%s (%s)', $machine->getNom(), $machine->getMachineToken()),
                'choice_attr' => static fn (Machine $machine): array => ['data-machine-token' => $machine->getMachineToken()],
                'placeholder' => 'admin_rfid_form.ph_machine',
                'help' => 'admin_rfid_form.help_target',
            ])
            ->add('accessPoint', EntityType::class, [
                'label' => 'admin_rfid_form.access_point',
                'class' => AccessPoint::class,
                'required' => false,
                // ⚠️ Un point archivé ne se PROPOSE pas — même partage que
                // partout ailleurs. Un lecteur déjà rattaché à un point archivé
                // continue de l'afficher : c'est `query_builder` qui filtre les
                // choix, pas la donnée existante.
                'query_builder' => static fn ($repository) => $repository->createQueryBuilder('ap')
                    ->andWhere('ap.archivedAt IS NULL')
                    ->orderBy('ap.nom', 'ASC'),
                'choice_label' => static fn (AccessPoint $point): string => $point->getNom(),
                'placeholder' => 'admin_rfid_form.ph_access_point',
                'help' => 'admin_rfid_form.help_target',
            ])
            ->add('isActive', CheckboxType::class, [
                'label' => 'admin_rfid_form.is_active',
                'help' => 'admin_rfid_form.help_is_active',
                'required' => false,
            ])
            ->add('save', SubmitType::class, ['label' => 'common.save']);

        /*
         * 🔴 **La règle « exactement une cible » se juge sur le FORMULAIRE, pas
         * sur l'entité — et c'est le piège de cette étape.** `setMachine()` et
         * `setAccessPoint()` s'excluent mutuellement : au moment où on
         * regarderait l'entité, une saisie qui coche les DEUX aurait déjà été
         * réduite à une seule, et la faute serait devenue invisible. Les données
         * soumises des deux champs, elles, sont intactes.
         *
         * ⚠️ L'erreur est posée sur le champ `machine` plutôt qu'en haut de page :
         * un message global oblige à chercher lequel des deux champs il vise.
         */
        $builder->addEventListener(FormEvents::POST_SUBMIT, static function (FormEvent $event): void {
            $form = $event->getForm();
            if (!$form->has('machine') || !$form->has('accessPoint')) {
                return;
            }

            $machine = $form->get('machine')->getData();
            $point = $form->get('accessPoint')->getData();

            if (($machine !== null) === ($point !== null)) {
                $form->get('machine')->addError(new FormError(
                    $machine === null
                        ? 'Choisissez ce que ce boîtier commande : une machine OU un point d’accès.'
                        : 'Un boîtier commande une machine OU un point d’accès, pas les deux.'
                ));
            }
        });
    }

    public function configureOptions(OptionsResolver $resolver): void
    {
        $resolver->setDefaults([
            'data_class' => RfidReader::class,
        ]);
    }
}
