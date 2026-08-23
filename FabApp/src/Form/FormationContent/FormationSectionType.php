<?php

namespace App\Form\FormationContent;

use App\Repository\SectionRepository;
use Symfony\Component\Form\AbstractType;
use Symfony\Component\Form\Extension\Core\Type\IntegerType;
use Symfony\Component\Form\Extension\Core\Type\TextareaType;
use Symfony\Component\Form\Extension\Core\Type\TextType;
use Symfony\Component\Form\Extension\Core\Type\UrlType;
use Symfony\Component\Form\FormBuilderInterface;
use Symfony\Component\OptionsResolver\OptionsResolver;
use Symfony\Component\Validator\Constraints as Assert;

/**
 * Une section du parcours guidé, création et édition (S148, J-22).
 *
 * 🔴 **Les cinq refus étaient des phrases françaises en dur listées en haut de
 * page.** L'écran gardait déjà ce qui avait été tapé — c'est un des rares qui le
 * faisait — mais l'opérateur devait relire cinq lignes pour savoir LEQUEL des
 * sept champs était en cause. Les contraintes le disent sur le champ, et le
 * catalogue `validators` les traduit dans les cinq langues.
 *
 * ⚠️ **La règle du préfixe réservé reste ici**, en `Regex` négative : un titre
 * qui commence par le préfixe des blocs de page ferait passer la section pour du
 * contenu de page et la ferait disparaître de la liste des sections.
 *
 * ⚠️ Les trois zones au format `a | b` ne sont pas typées plus finement : leur
 * format est une convention de saisie que le contrôleur découpe, et le seul refus
 * qui compte — « au moins une étape » — dépend du résultat du découpage, donc il
 * reste là-bas et se pose sur `steps`.
 */
final class FormationSectionType extends AbstractType
{
    public function buildForm(FormBuilderInterface $builder, array $options): void
    {
        $builder
            ->add('titre', TextType::class, [
                'label' => 'formation_section.field_title',
                'empty_data' => '',
                'constraints' => [
                    new Assert\NotBlank(message: 'Le titre de la section est obligatoire.'),
                    new Assert\Length(max: 255, maxMessage: 'Le titre ne doit pas dépasser {{ limit }} caractères.'),
                    new Assert\Regex(
                        pattern: '/^' . preg_quote(SectionRepository::PAGE_BLOCK_PREFIX, '/') . '/',
                        match: false,
                        message: 'Ce préfixe de titre est réservé au contenu de la page.',
                    ),
                ],
            ])
            ->add('ordre', IntegerType::class, [
                'label' => 'formation_section.field_order',
                'attr' => ['min' => 1],
                'constraints' => [new Assert\Positive(message: "L'ordre doit être un entier positif.")],
            ])
            ->add('videoUrl', UrlType::class, [
                'label' => 'formation_section.field_video',
                'required' => false,
                'row_attr' => ['class' => 'full'],
                'constraints' => [
                    new Assert\Length(max: 255, maxMessage: 'Le lien ne doit pas dépasser {{ limit }} caractères.'),
                    new Assert\Url(message: "Cette adresse n'est pas une URL valide."),
                ],
            ])
            ->add('intro', TextareaType::class, [
                'label' => 'formation_section.field_intro',
                'empty_data' => '',
                'row_attr' => ['class' => 'full'],
                'constraints' => [new Assert\NotBlank(message: "Le texte d'introduction est obligatoire.")],
            ])
            ->add('objectives', TextareaType::class, [
                'label' => 'formation_section.field_objectives',
                'help' => 'formation_section.objectives_help',
                'required' => false,
                'row_attr' => ['class' => 'full'],
            ])
            ->add('steps', TextareaType::class, [
                'label' => 'formation_section.field_steps',
                'help' => 'formation_section.steps_help',
                'required' => false,
                'row_attr' => ['class' => 'full'],
            ])
            ->add('callouts', TextareaType::class, [
                'label' => 'formation_section.field_callouts',
                'help' => 'formation_section.callouts_help',
                'required' => false,
                'row_attr' => ['class' => 'full'],
            ]);
    }

    public function configureOptions(OptionsResolver $resolver): void
    {
        $resolver->setDefaults([
            'data_class' => null,
            'csrf_token_id' => 'formation_section',
        ]);
    }
}
