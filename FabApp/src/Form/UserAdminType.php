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
    /**
     * L'ordre et le découpage de l'écran — motif `SECTIONS` (S151, R3), déroulé
     * par `site/_form_sections.html.twig`. Une seule liste, dans le formulaire,
     * au lieu d'une par gabarit qui rendait le même type.
     *
     * 🔴 **Ce gabarit dessinait ses rangées à la main** (`form_label` +
     * `form_widget`), donc il sautait le thème : **5 champs obligatoires et 0
     * mention « requis »** à l'écran, mesuré avant conversion. Et ses deux cases
     * étaient écrites à la main aussi, ce qui saute `form_help()` — le thème a un
     * bloc `checkbox_row` qui fait les deux correctement.
     */
    public const SECTIONS = [
        [
            'title' => 'admin_user_form.section_identity',
            'fields' => ['firstName', 'lastName', 'email', 'username'],
        ],
        [
            'title' => 'admin_user_form.section_access',
            'fields' => ['plainPassword', 'confirmPassword', 'role', 'statut'],
        ],
        [
            'title' => 'admin_user_form.section_badge',
            'fold' => true,
            'fields' => ['identifiantRfid', 'numeroId'],
        ],
        [
            'title' => 'admin_user_form.section_prefs',
            'fold' => true,
            'fields' => ['notificationEmail', 'notificationPush', 'theme', 'langue'],
        ],
    ];

    public function buildForm(FormBuilderInterface $builder, array $options): void
    {
        /** @var array<string, string> $localeChoices code => display name */
        $localeChoices = $options['locale_choices'];

        $builder
            ->add('email', EmailType::class, [
                'label' => 'admin_user_form.email',
                'empty_data' => '',
                'constraints' => [
                    new Assert\NotBlank(message: 'L’email est obligatoire.'),
                    new Assert\Email(message: 'L’email est invalide.'),
                    new Assert\Length(max: 255, maxMessage: 'L’email ne doit pas dépasser {{ limit }} caractères.'),
                ],
            ])
            ->add('username', TextType::class, [
                'label' => 'admin_user_form.username',
                'empty_data' => '',
                'constraints' => [
                    new Assert\NotBlank(message: 'Le username est obligatoire.'),
                    new Assert\Length(max: 255, maxMessage: 'Le username ne doit pas dépasser {{ limit }} caractères.'),
                ],
            ])
            ->add('firstName', TextType::class, [
                'label' => 'admin_user_form.first_name',
                'required' => false,
                'constraints' => [new Assert\Length(max: 255, maxMessage: 'Le prénom ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('lastName', TextType::class, [
                'label' => 'register.lastname',
                'required' => false,
                'constraints' => [new Assert\Length(max: 255, maxMessage: 'Le nom ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('plainPassword', PasswordType::class, [
                'label' => 'admin_user_form.plain_password',
                'mapped' => false,
                'empty_data' => '',
                'constraints' => [
                    new Assert\NotBlank(message: 'Le mot de passe est obligatoire.'),
                    new Assert\Length(min: 8, max: 4096, minMessage: 'Le mot de passe doit contenir au moins {{ limit }} caractères.'),
                ],
            ])
            ->add('confirmPassword', PasswordType::class, [
                'label' => 'admin_user_form.confirm_password',
                'mapped' => false,
                'empty_data' => '',
                'constraints' => [new Assert\NotBlank(message: 'La confirmation est obligatoire.')],
            ])
            // 🔴 **S159 — ce champ écrit une APPARTENANCE, pas un rôle.** Il garde
            // son nom `role` parce que le renommer casserait les gabarits et les
            // jetons pour une valeur d'exactitude interne ; mais son LIBELLÉ dit
            // « groupe », parce que c'est ce que l'opérateur choisit et ce que le
            // contrôleur écrit. Un champ dont l'étiquette nomme autre chose que ce
            // qu'il fait est la première marche vers un écran qui ment.
            ->add('role', ChoiceType::class, [
                'label' => 'admin_user_form.role',
                'mapped' => false,
                'choices' => $options['role_choices'],
                'placeholder' => 'admin_user_form.ph_role',
                'constraints' => [
                    new Assert\NotBlank(message: 'Le groupe est obligatoire.'),
                    new Assert\Choice(choices: array_values($options['role_choices']), message: 'Groupe invalide.'),
                ],
            ])
            ->add('identifiantRfid', TextType::class, [
                'label' => 'admin_user_form.rfid',
                'required' => false,
                'constraints' => [new Assert\Length(max: 255, maxMessage: 'L’identifiant RFID ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('numeroId', TextType::class, [
                'label' => 'admin_user_form.numero_id',
                'required' => false,
                'constraints' => [new Assert\Length(max: 100, maxMessage: 'Le numéro ID ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('statut', ChoiceType::class, [
                'label' => 'form.status',
                'choices' => [
                    'admin_user_form.choice_active' => 'actif',
                    'admin_user_form.choice_inactive' => 'inactif',
                ],
                'constraints' => [new Assert\Choice(choices: ['actif', 'inactif'], message: 'Statut invalide.')],
            ])
            ->add('notificationEmail', CheckboxType::class, [
                'label' => 'admin_user_form.notification_email',
                'required' => false,
            ])
            ->add('notificationPush', CheckboxType::class, [
                'label' => 'admin_user_form.notification_push',
                'required' => false,
            ])
            // 'rappelReservation' used to sit here. It was writable from both this
            // form and the profile, and read by nothing — so switching it off never
            // stopped a single mail. Reminder opt-outs now live in
            // USER_NOTIFICATION_OPTOUT, which the person owns themselves; the old
            // values were migrated across in Version20260729100000.
            ->add('theme', ChoiceType::class, [
                'label' => 'admin_user_form.theme',
                'choices' => [
                    'admin_user_form.choice_system' => 'system',
                    'admin_user_form.choice_light' => 'light',
                    'admin_user_form.choice_dark' => 'dark',
                ],
                'constraints' => [new Assert\Choice(choices: ['system', 'light', 'dark'], message: 'Thème invalide.')],
            ])
            // Choices and constraint both come from `locale_choices`, passed in by the
            // controller from LocaleCatalog. The hardcoded pair that used to be here
            // meant an admin could not create a German, Spanish or Italian account.
            ->add('langue', ChoiceType::class, [
                'label' => 'admin_user_form.langue',
                'choices' => array_flip($localeChoices),
                'constraints' => [new Assert\Choice(choices: array_keys($localeChoices), message: 'Langue invalide.')],
            ])
            ->add('save', SubmitType::class, ['label' => 'admin_user_form.submit_new']);
    }

    public function configureOptions(OptionsResolver $resolver): void
    {
        $resolver->setDefaults([
            'data_class' => Utilisateur::class,
            'role_choices' => [],
        ]);

        // Required, not defaulted: a default would quietly offer one language to a
        // caller that forgot to pass the list — which is exactly the bug this
        // replaced. Failing loudly at build time is the point.
        $resolver->setRequired('locale_choices');
        $resolver->setAllowedTypes('locale_choices', 'array');

        $resolver->setAllowedTypes('role_choices', 'array');
    }
}
