<?php

namespace App\Form;

use App\Entity\Utilisateur;
use Symfony\Component\Form\AbstractType;
use Symfony\Component\Form\Extension\Core\Type\CheckboxType;
use Symfony\Component\Form\Extension\Core\Type\ChoiceType;
use Symfony\Component\Form\Extension\Core\Type\EmailType;
use Symfony\Component\Form\Extension\Core\Type\PasswordType;
use Symfony\Component\Form\Extension\Core\Type\SubmitType;
use Symfony\Component\Form\Extension\Core\Type\TextType;
use Symfony\Component\Form\FormBuilderInterface;
use Symfony\Component\OptionsResolver\OptionsResolver;
use Symfony\Component\Validator\Constraints as Assert;

final class UserAdminType extends AbstractType
{
    public function buildForm(FormBuilderInterface $builder, array $options): void
    {
        $builder
            ->add('email', EmailType::class, [
                'label' => 'Email',
                'empty_data' => '',
                'constraints' => [
                    new Assert\NotBlank(message: 'L’email est obligatoire.'),
                    new Assert\Email(message: 'L’email est invalide.'),
                    new Assert\Length(max: 255, maxMessage: 'L’email ne doit pas dépasser {{ limit }} caractères.'),
                ],
            ])
            ->add('username', TextType::class, [
                'label' => 'Username',
                'empty_data' => '',
                'constraints' => [
                    new Assert\NotBlank(message: 'Le username est obligatoire.'),
                    new Assert\Length(max: 255, maxMessage: 'Le username ne doit pas dépasser {{ limit }} caractères.'),
                ],
            ])
            ->add('firstName', TextType::class, [
                'label' => 'Prénom',
                'required' => false,
                'constraints' => [new Assert\Length(max: 255, maxMessage: 'Le prénom ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('lastName', TextType::class, [
                'label' => 'Nom',
                'required' => false,
                'constraints' => [new Assert\Length(max: 255, maxMessage: 'Le nom ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('plainPassword', PasswordType::class, [
                'label' => 'Mot de passe temporaire',
                'mapped' => false,
                'empty_data' => '',
                'constraints' => [
                    new Assert\NotBlank(message: 'Le mot de passe est obligatoire.'),
                    new Assert\Length(min: 8, max: 4096, minMessage: 'Le mot de passe doit contenir au moins {{ limit }} caractères.'),
                ],
            ])
            ->add('confirmPassword', PasswordType::class, [
                'label' => 'Confirmation',
                'mapped' => false,
                'empty_data' => '',
                'constraints' => [new Assert\NotBlank(message: 'La confirmation est obligatoire.')],
            ])
            ->add('role', ChoiceType::class, [
                'label' => 'Rôle',
                'mapped' => false,
                'choices' => $options['role_choices'],
                'placeholder' => 'Choisir un rôle',
                'constraints' => [
                    new Assert\NotBlank(message: 'Le rôle est obligatoire.'),
                    new Assert\Choice(choices: array_values($options['role_choices']), message: 'Rôle invalide.'),
                ],
            ])
            ->add('identifiantRfid', TextType::class, [
                'label' => 'Identifiant RFID',
                'required' => false,
                'constraints' => [new Assert\Length(max: 255, maxMessage: 'L’identifiant RFID ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('numeroId', TextType::class, [
                'label' => 'Numéro ID',
                'required' => false,
                'constraints' => [new Assert\Length(max: 100, maxMessage: 'Le numéro ID ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('statut', ChoiceType::class, [
                'label' => 'Statut',
                'choices' => [
                    'Actif' => 'actif',
                    'Inactif' => 'inactif',
                ],
                'constraints' => [new Assert\Choice(choices: ['actif', 'inactif'], message: 'Statut invalide.')],
            ])
            ->add('notificationEmail', CheckboxType::class, [
                'label' => 'Notification email',
                'required' => false,
            ])
            ->add('notificationPush', CheckboxType::class, [
                'label' => 'Notification push',
                'required' => false,
            ])
            ->add('rappelReservation', CheckboxType::class, [
                'label' => 'Rappel réservation',
                'required' => false,
            ])
            ->add('theme', ChoiceType::class, [
                'label' => 'Thème',
                'choices' => [
                    'Système' => 'system',
                    'Clair' => 'light',
                    'Sombre' => 'dark',
                ],
                'constraints' => [new Assert\Choice(choices: ['system', 'light', 'dark'], message: 'Thème invalide.')],
            ])
            ->add('langue', ChoiceType::class, [
                'label' => 'Langue',
                'choices' => [
                    'Français' => 'fr',
                    'Anglais' => 'en',
                ],
                'constraints' => [new Assert\Choice(choices: ['fr', 'en'], message: 'Langue invalide.')],
            ])
            ->add('save', SubmitType::class, ['label' => 'Créer l’utilisateur']);
    }

    public function configureOptions(OptionsResolver $resolver): void
    {
        $resolver->setDefaults([
            'data_class' => Utilisateur::class,
            'role_choices' => [],
        ]);

        $resolver->setAllowedTypes('role_choices', 'array');
    }
}
