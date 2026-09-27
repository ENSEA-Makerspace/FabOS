<?php

namespace App\Form\Admin;

use App\Identity\AttributeMapping;
use App\Identity\AuthProvider;
use Symfony\Component\Form\AbstractType;
use Symfony\Component\Form\Extension\Core\Type\CheckboxType;
use Symfony\Component\Form\Extension\Core\Type\ChoiceType;
use Symfony\Component\Form\Extension\Core\Type\TextType;
use Symfony\Component\Form\Extension\Core\Type\UrlType;
use Symfony\Component\Form\FormBuilderInterface;
use Symfony\Component\OptionsResolver\OptionsResolver;
use Symfony\Component\Validator\Constraints as Assert;

/**
 * S196 — un fournisseur de connexion, dans « Connexion & annuaires ».
 * Remplace `NetworkProviderType` (S148), qui vivait dans l'écran de la fédération.
 *
 * 🔴 `secretEnv` est le NOM d'une variable d'environnement, jamais le secret :
 * le motif (majuscules, chiffres, `_`) refuse qu'on y colle une valeur.
 * ⚠️ La correspondance des attributs est FACULTATIVE champ par champ : vide =
 * la valeur du préréglage, affichée en indication.
 * ⚠️ La clé est figée une fois créée : les liens `EXTERNAL_IDENTITY` la portent.
 */
final class AuthProviderType extends AbstractType
{
    public function buildForm(FormBuilderInterface $builder, array $options): void
    {
        $preset = AttributeMapping::preset((string) $options['preset']);
        $builder
            ->add('label', TextType::class, [
                'label' => 'identity.field.label',
                'help' => 'identity.help.label',
                'empty_data' => '',
                'constraints' => [new Assert\NotBlank(), new Assert\Length(max: 120)],
            ])
            ->add('key', TextType::class, [
                'label' => 'identity.field.key',
                'help' => 'identity.help.key',
                'empty_data' => '',
                'disabled' => $options['editing'],
                'constraints' => $options['editing'] ? [] : [
                    new Assert\NotBlank(),
                    new Assert\Regex(pattern: '/^[a-z][a-z0-9_]{1,79}$/', message: 'identity.invalid.key'),
                ],
            ])
            ->add('issuer', UrlType::class, [
                'label' => 'identity.field.issuer',
                'help' => 'identity.help.issuer',
                'default_protocol' => 'https',
                'empty_data' => '',
                'constraints' => [new Assert\NotBlank(), new Assert\Regex(pattern: '#^https://#', message: 'identity.invalid.issuer')],
            ])
            ->add('clientId', TextType::class, [
                'label' => 'identity.field.client_id',
                'empty_data' => '',
                'constraints' => [new Assert\NotBlank(), new Assert\Length(max: 255)],
            ])
            ->add('secretEnv', TextType::class, [
                'label' => 'identity.field.secret_env',
                'help' => 'identity.help.secret_env',
                'empty_data' => '',
                'constraints' => [new Assert\NotBlank(), new Assert\Regex(pattern: '/^[A-Z][A-Z0-9_]{2,119}$/', message: 'identity.invalid.secret_env')],
            ])
            ->add('scopes', TextType::class, [
                'label' => 'identity.field.scopes',
                'help' => 'identity.help.scopes',
                'empty_data' => 'openid profile email',
                'required' => false,
            ])
            ->add('preset', ChoiceType::class, [
                'label' => 'identity.field.preset',
                'help' => 'identity.help.preset',
                'choices' => array_combine(
                    array_map(static fn (string $p): string => 'identity.preset.' . $p, AttributeMapping::presetsFor(AuthProvider::KIND_OIDC)),
                    AttributeMapping::presetsFor(AuthProvider::KIND_OIDC),
                ),
            ])
            ->add('passwordUrl', UrlType::class, [
                'label' => 'identity.field.password_url',
                'help' => 'identity.help.password_url',
                'required' => false,
                'default_protocol' => 'https',
                'empty_data' => '',
                'constraints' => [new Assert\Regex(pattern: '#^(https://\S+)?$#', message: 'identity.invalid.password_url')],
            ])
            ->add('trustEmail', CheckboxType::class, [
                'label' => 'identity.field.trust_email',
                'help' => 'identity.help.trust_email',
                'required' => false,
            ])
            ->add('enabled', CheckboxType::class, [
                'label' => 'identity.field.enabled',
                'help' => 'identity.help.enabled',
                'required' => false,
            ]);

        foreach (AttributeMapping::FIELDS as $field) {
            $builder->add('map_' . $field, TextType::class, [
                'label' => 'identity.profile.' . $field,
                'required' => false,
                'empty_data' => '',
                'attr' => ['placeholder' => $preset[$field] ?? '—'],
                'constraints' => [new Assert\Length(max: 120)],
            ]);
        }
    }

    public function configureOptions(OptionsResolver $resolver): void
    {
        $resolver->setDefaults([
            'data_class' => null,
            'csrf_token_id' => 'admin_identity_provider',
            'editing' => false,
            'preset' => AttributeMapping::DEFAULT_PRESET,
        ]);
        $resolver->setAllowedTypes('editing', 'bool');
        $resolver->setAllowedTypes('preset', 'string');
    }
}
