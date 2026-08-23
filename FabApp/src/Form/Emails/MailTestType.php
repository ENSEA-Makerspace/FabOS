<?php

namespace App\Form\Emails;

use Symfony\Component\Form\AbstractType;
use Symfony\Component\Form\Extension\Core\Type\EmailType;
use Symfony\Component\Form\FormBuilderInterface;
use Symfony\Component\OptionsResolver\OptionsResolver;
use Symfony\Component\Validator\Constraints as Assert;
use Symfony\Contracts\Translation\TranslatorInterface;

/**
 * `/admin/emails`, carte « Envoyer un test » (S147, J-22).
 *
 * ⚠️ **Un seul champ, et c'est celui qui gagne le plus à être validé** : le
 * contrôleur envoyait à ce qu'on lui donnait, donc une adresse mal tapée
 * produisait un échec SMTP présenté comme une panne de configuration. Refusée
 * ici, l'erreur dit ce qu'elle est.
 */
final class MailTestType extends AbstractType
{
    public function __construct(private readonly TranslatorInterface $translator)
    {
    }

    public function buildForm(FormBuilderInterface $builder, array $options): void
    {
        $builder->add('test_recipient', EmailType::class, [
            'label' => 'admin_emails.test_recipient',
            'help' => 'admin_emails.test_help',
            'empty_data' => '',
            'attr' => ['placeholder' => $this->translator->trans('admin_emails.test_placeholder')],
            'required' => false,
            // ⚠️ **Pas de `NotBlank`, et c'est voulu** : laissé vide, le contrôleur
            // envoie à l'adresse de l'opérateur connecté — un raccourci qui existait
            // avant et qu'on ne retire pas au passage. `Assert\Email` ignore une
            // valeur vide, donc seule une adresse mal tapée est refusée.
            'constraints' => [new Assert\Email(message: 'L’email est invalide.')],
        ]);
    }

    public function configureOptions(OptionsResolver $resolver): void
    {
        $resolver->setDefaults([
            'data_class' => null,
            'csrf_token_id' => 'emails_test',
        ]);
    }
}
