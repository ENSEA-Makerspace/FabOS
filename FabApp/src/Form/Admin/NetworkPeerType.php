<?php

namespace App\Form\Admin;

use Symfony\Component\Form\AbstractType;
use Symfony\Component\Form\Extension\Core\Type\TextType;
use Symfony\Component\Form\Extension\Core\Type\TextareaType;
use Symfony\Component\Form\Extension\Core\Type\UrlType;
use Symfony\Component\Form\FormBuilderInterface;
use Symfony\Component\OptionsResolver\OptionsResolver;
use Symfony\Component\Validator\Constraints as Assert;

/**
 * `/admin/network`, faire confiance à un pair (S148, J-22).
 *
 * 🔴 **Les deux vérifications du contrôleur étaient une exception jetée dont le
 * message partait en flash, et la page redirigeait** — les quatre champs, dont
 * une clé publique en base64 qu'on venait de coller, étaient perdus. Elles sont
 * des contraintes : l'UUID par une expression régulière, la clé publique par un
 * rappel qui décode et compte les octets. Le contrôleur garde la même règle au
 * moment d'écrire ; ce qui change, c'est qu'on la voit avant.
 */
final class NetworkPeerType extends AbstractType
{
    public function buildForm(FormBuilderInterface $builder, array $options): void
    {
        $builder
            ->add('instanceUuid', TextType::class, [
                'label' => 'network.instanceUuid',
                'empty_data' => '',
                'constraints' => [
                    new Assert\NotBlank(message: "L'identifiant de l'instance est obligatoire."),
                    new Assert\Regex(pattern: '/^[0-9a-f-]{36}$/i', message: "Cet identifiant d'instance n'est pas un UUID."),
                ],
            ])
            ->add('peerOrigin', UrlType::class, [
                'label' => 'network.peerOrigin',
                'empty_data' => '',
                'constraints' => [
                    new Assert\NotBlank(message: "L'origine est obligatoire."),
                    new Assert\Url(message: "Cette adresse n'est pas une URL valide."),
                ],
            ])
            ->add('keyId', TextType::class, [
                'label' => 'network.keyId',
                'empty_data' => '',
                'constraints' => [new Assert\NotBlank(message: "L'identifiant de clé est obligatoire.")],
            ])
            ->add('publicKey', TextareaType::class, [
                'label' => 'network.publicKey',
                'empty_data' => '',
                'row_attr' => ['class' => 'full'],
                'constraints' => [
                    new Assert\NotBlank(message: 'La clé publique est obligatoire.'),
                    new Assert\Callback(static function (?string $value, $context): void {
                        if ($value === null || $value === '') {
                            return;
                        }
                        $raw = base64_decode($value, true);
                        if ($raw === false || \strlen($raw) !== SODIUM_CRYPTO_SIGN_PUBLICKEYBYTES) {
                            $context->buildViolation('Cette clé publique n’est pas une clé de signature valide en base64.')->addViolation();
                        }
                    }),
                ],
            ]);
    }

    public function configureOptions(OptionsResolver $resolver): void
    {
        $resolver->setDefaults(['data_class' => null, 'csrf_token_id' => 'network_settings']);
    }
}
