<?php

namespace App\Form;

use App\Entity\Loan;
use App\Entity\LoanableItem;
use App\Entity\Utilisateur;
use Symfony\Bridge\Doctrine\Form\Type\EntityType;
use Symfony\Component\Form\AbstractType;
use Symfony\Component\Form\Extension\Core\Type\DateTimeType;
use Symfony\Component\Form\Extension\Core\Type\DateType;
use Symfony\Component\Form\Extension\Core\Type\EmailType;
use Symfony\Component\Form\Extension\Core\Type\SubmitType;
use Symfony\Component\Form\Extension\Core\Type\TextareaType;
use Symfony\Component\Form\Extension\Core\Type\TextType;
use Symfony\Component\Form\FormBuilderInterface;
use Symfony\Component\OptionsResolver\OptionsResolver;
use Symfony\Component\Validator\Constraints as Assert;

/**
 * Staff checkout form for a new Loan. The borrower is either a registered user
 * (dropdown) or the free-text contact fields when they are not one.
 */
final class LoanAdminType extends AbstractType
{
    /**
     * L'ordre et le découpage de l'écran — motif `SECTIONS` (S151, R3), déroulé
     * par `site/_form_sections.html.twig`. Une seule liste, dans le formulaire,
     * au lieu d'une par gabarit qui rendait le même type.
     *
     * ⚠️ Les quatre champs « emprunteur » sont un seul choix à deux formes : un
     * membre du lab (`borrower`), ou quelqu'un de passage qu'on décrit à la main.
     * Ils vont donc ensemble, sous un titre qui pose la question.
     */
    public const SECTIONS = [
        [
            'title' => 'admin_loan_form.section_loan',
            'fields' => ['item', 'dateTaken', 'expectedReturnDate'],
        ],
        [
            'title' => 'admin_loan_form.section_borrower',
            'fields' => ['borrower', 'borrowerName', 'borrowerEmail', 'borrowerPhone'],
        ],
        [
            'title' => 'admin_loan_form.section_notes',
            'fold' => true,
            'fields' => ['conditionOut', 'notes'],
        ],
    ];

    public function buildForm(FormBuilderInterface $builder, array $options): void
    {
        $builder
            ->add('item', EntityType::class, [
                'label' => 'Objet emprunté',
                'class' => LoanableItem::class,
                'choice_label' => 'name',
                'placeholder' => '— Choisir un objet —',
                // ⚠️ S133 — an archived object is not offered for a NEW loan. Loans
                // already out against it keep resolving; this is the only place the
                // archive has to bite, because everything else is history.
                'query_builder' => static fn ($repo) => $repo->createQueryBuilder('i')
                    ->andWhere('i.archivedAt IS NULL')
                    ->orderBy('i.name', 'ASC'),
                'constraints' => [new Assert\NotNull(message: 'Choisissez un objet.')],
            ])
            ->add('borrower', EntityType::class, [
                'label' => 'Emprunteur inscrit (optionnel)',
                'class' => Utilisateur::class,
                'choice_label' => fn (Utilisateur $u) => $u->getDisplayName() . ' (' . $u->getEmail() . ')',
                'placeholder' => '— Emprunteur non inscrit —',
                'required' => false,
                'query_builder' => static fn ($repo) => $repo->createQueryBuilder('u')->orderBy('u.firstName', 'ASC'),
            ])
            ->add('borrowerName', TextType::class, [
                'label' => 'Nom (si non inscrit)',
                'required' => false,
                'constraints' => [new Assert\Length(max: 180)],
            ])
            ->add('borrowerEmail', EmailType::class, [
                'label' => 'Email (si non inscrit)',
                'required' => false,
                'constraints' => [new Assert\Length(max: 180)],
            ])
            ->add('borrowerPhone', TextType::class, [
                'label' => 'Téléphone (si non inscrit)',
                'required' => false,
                'constraints' => [new Assert\Length(max: 60)],
            ])
            ->add('dateTaken', DateTimeType::class, [
                'label' => 'Date de prêt',
                'widget' => 'single_text',
                'input' => 'datetime_immutable',
                'constraints' => [new Assert\NotNull()],
            ])
            ->add('expectedReturnDate', DateType::class, [
                'label' => 'Retour prévu (optionnel)',
                'widget' => 'single_text',
                'input' => 'datetime_immutable',
                'required' => false,
            ])
            ->add('conditionOut', TextareaType::class, [
                'row_attr' => ['class' => 'full'],
                'label' => 'État au départ',
                'required' => false,
                'constraints' => [new Assert\Length(max: 1000)],
            ])
            ->add('notes', TextareaType::class, [
                'row_attr' => ['class' => 'full'],
                'label' => 'Informations complémentaires',
                'required' => false,
                'constraints' => [new Assert\Length(max: 1000)],
            ])
            ->add('save', SubmitType::class, ['label' => 'Enregistrer le prêt']);
    }

    public function configureOptions(OptionsResolver $resolver): void
    {
        $resolver->setDefaults(['data_class' => Loan::class]);
    }
}
