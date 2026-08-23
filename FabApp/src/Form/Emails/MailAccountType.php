<?php

namespace App\Form\Emails;

use Symfony\Component\Form\AbstractType;
use Symfony\Component\Form\Extension\Core\Type\CheckboxType;
use Symfony\Component\Form\Extension\Core\Type\EmailType;
use Symfony\Component\Form\Extension\Core\Type\TextType;
use Symfony\Component\Form\Extension\Core\Type\UrlType;
use Symfony\Component\Form\FormBuilderInterface;
use Symfony\Component\OptionsResolver\OptionsResolver;
use Symfony\Component\Validator\Constraints as Assert;
use Symfony\Contracts\Translation\TranslatorInterface;

/**
 * `/admin/emails`, carte « Compte d'envoi » (S147, J-22).
 *
 * ⚠️ **Les adresses sont enfin validées, et elles ne l'étaient pas.** Le gabarit
 * posait `type="email"`, ce qui n'engage que le navigateur : le contrôleur lisait
 * la valeur telle quelle. Une adresse d'expéditeur invalide n'était donc refusée
 * par personne avant que le premier envoi n'échoue, loin de l'écran qui l'avait
 * acceptée.
 *
 * ⚠️ Trois aides portent du balisage voulu (`|raw` dans le gabarit) : `help_html`
 * les préserve. Sans lui, un `<code>` serait apparu littéralement à l'écran.
 */
final class MailAccountType extends AbstractType
{
    public function __construct(private readonly TranslatorInterface $translator)
    {
    }

    public function buildForm(FormBuilderInterface $builder, array $options): void
    {
        $builder
            ->add('mail_transport_dsn', TextType::class, [
                'label' => 'admin_emails.dsn',
                'help' => 'admin_emails.dsn_help',
                'help_html' => true,
                'required' => false,
                'empty_data' => '',
                'row_attr' => ['class' => 'full'],
                'attr' => [
                    'placeholder' => $this->translator->trans('admin_emails.dsn_placeholder'),
                    'autocomplete' => 'off',
                    'spellcheck' => 'false',
                ],
            ])
            ->add('mail_from_address', EmailType::class, [
                'label' => 'admin_emails.from_address',
                'required' => false,
                'empty_data' => '',
                'attr' => ['placeholder' => 'fablab@exemple.fr'],
                'constraints' => [new Assert\Email(message: 'L’email est invalide.')],
            ])
            ->add('mail_from_name', TextType::class, [
                'label' => 'admin_emails.from_name',
                'help' => 'admin_emails.from_name_help',
                'required' => false,
                'empty_data' => '',
                'attr' => ['maxlength' => 120, 'placeholder' => $this->translator->trans('admin_emails.from_name_placeholder')],
                'constraints' => [new Assert\Length(max: 120, maxMessage: 'Ce champ ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('mail_reply_to', EmailType::class, [
                'label' => 'admin_emails.reply_to',
                'required' => false,
                'empty_data' => '',
                'attr' => ['placeholder' => 'contact@exemple.fr'],
                'constraints' => [new Assert\Email(message: 'L’email est invalide.')],
            ])
            ->add('public_base_url', UrlType::class, [
                'label' => 'admin_emails.public_url',
                'help' => 'admin_emails.public_url_help',
                'help_html' => true,
                'required' => false,
                'empty_data' => '',
                'row_attr' => ['class' => 'full'],
                'attr' => ['placeholder' => 'https://fablab.exemple.fr', 'spellcheck' => 'false'],
            ])
            ->add('mail_paused', CheckboxType::class, [
                'label' => 'admin_emails.pause_label',
                'help' => 'admin_emails.pause_help',
                'help_html' => true,
                'required' => false,
                'row_attr' => ['class' => 'full'],
            ]);
    }

    public function configureOptions(OptionsResolver $resolver): void
    {
        $resolver->setDefaults([
            'data_class' => null,
            // Voir `GeneralSettingsType` : un identifiant propre retombe sur le jeton
            // de SESSION, donc ce formulaire se teste par sonde.
            'csrf_token_id' => 'emails_account',
        ]);
    }
}
